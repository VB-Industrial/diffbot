#include "ruka_hardware.hpp"
#include "ruka_sensor_hardware.hpp"
#include "ruka_joints.h"

#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <iomanip> 

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


#include "cyphal/allocators/o1/o1_allocator.h"
#include "cyphal/cyphal.h"
#include "cyphal/providers/LinuxCAN.h"
#include "cyphal/subscriptions/subscription.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "reg/udral/physics/kinematics/rotation/Planar_0_1.h"
#include "reg/udral/physics/kinematics/cartesian/Twist_0_1.h"
#include "reg/udral/physics/kinematics/cartesian/State_0_1.h"

#include <uavcan/_register/Access_1_0.h>
#include <uavcan/_register/List_1_0.h>


TYPE_ALIAS(Twist, reg_udral_physics_kinematics_cartesian_Twist_0_1)
TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(JS_msg, reg_udral_physics_kinematics_rotation_Planar_0_1)
TYPE_ALIAS(State, reg_udral_physics_kinematics_cartesian_State_0_1)

TYPE_ALIAS(RegisterListRequest, uavcan_register_List_Request_1_0)
TYPE_ALIAS(RegisterListResponse, uavcan_register_List_Response_1_0)

TYPE_ALIAS(RegisterAccessRequest, uavcan_register_Access_Request_1_0)
TYPE_ALIAS(RegisterAccessResponse, uavcan_register_Access_Response_1_0)

std::byte buffer[sizeof(CyphalInterface) + sizeof(LinuxCAN) + sizeof(O1Allocator)];

void error_handler() {std::cout << "error" << std::endl; std::exit(EXIT_FAILURE);}
uint64_t micros_64() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
UtilityConfig utilities(micros_64, error_handler);

std::shared_ptr<CyphalInterface> cy_interface;

uint32_t uptime = 0;
void heartbeat() {
    static CanardTransferID hbeat_transfer_id = 0;
    HBeat::Type heartbeat_msg = {.uptime = uptime, .health = {uavcan_node_Health_1_0_NOMINAL}, .mode = {uavcan_node_Mode_1_0_OPERATIONAL}};
    cy_interface->send_msg<HBeat>(&heartbeat_msg, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, &hbeat_transfer_id);
    uptime += 1;
}

static float j_pos[6] = {0.0};
static float j_vel[6] = {0.0};
static float j_eff[6] = {0.0};

class JSReader_01: public AbstractSubscription<JS_msg> {
public:
    JSReader_01(InterfacePtr interface): AbstractSubscription<JS_msg>(
      interface,
      AGENT_JS_SUB_PORT
    ) {};
    void handler(const reg_udral_physics_kinematics_rotation_Planar_0_1& js_read, CanardRxTransfer* transfer) override {
      
        //std::cout << "Node id: " << +transfer->metadata.remote_node_id << std::endl;
        j_pos[transfer->metadata.remote_node_id-1] = js_read.angular_position.radian;
        j_vel[transfer->metadata.remote_node_id-1] = js_read.angular_velocity.radian_per_second;
        j_eff[transfer->metadata.remote_node_id-1] = js_read.angular_acceleration.radian_per_second_per_second;

        //std::cout << "pos: " << js_read.angular_position.radian << std::endl;
        //std::cout << "vel: " << js_read.angular_velocity.radian_per_second << std::endl;
       // std::cout << "eff: " << js_read.angular_acceleration.radian_per_second_per_second << std::endl;
        // j_pos_0 = js_read.angular_position.radian;
        // j_pos_1 = js_read.angular_velocity.radian_per_second;
        // j_pos_2 = js_read.angular_acceleration.radian_per_second_per_second;
    }
};
JSReader_01 * JS_reader_01;


class RegisterAccessReader : public AbstractSubscription<RegisterAccessResponse> {
public:
    RegisterAccessReader(InterfacePtr interface): AbstractSubscription<RegisterAccessResponse>(
        interface,
        uavcan_register_Access_1_0_FIXED_PORT_ID_,
        CanardTransferKindResponse
    ) {};
    void handler(const uavcan_register_Access_Response_1_0& reg_resp, CanardRxTransfer* transfer) override
    {

      std::cout << "SERVICE RESPONSE" << std::endl;
      //std::cout << +reg_resp.value.natural8.value.elements[0] <<": value"<<std::endl;
    };
};

RegisterAccessReader * RegAccessReader;

static CanardTransferID reg_access_transfer = 0;

void serv_send(CanardNodeID node_id) {
    reg_access_transfer++;
    RegisterAccessRequest::Type reg_access_request = {0};

    sprintf((char*)reg_access_request.name.name.elements, "test_reg");
    reg_access_request.name.name.count = strlen((char*)reg_access_request.name.name.elements);

    uavcan_register_Value_1_0 value = {};
    value._tag_ = 4;

    uavcan_primitive_array_Integer64_1_0 result = {};
    result.value.elements[0] = 2; //VALUE HERE
    result.value.count = 1;

    value.integer64 = result;
    reg_access_request.value = value;

        cy_interface->send_request<RegisterAccessRequest>(
        &reg_access_request,
        uavcan_register_Access_1_0_FIXED_PORT_ID_,
        &reg_access_transfer,
        node_id
    );
}

static CanardTransferID int_transfer_id = 0;

void send_JS(CanardNodeID node_id, float pos, float vel, float eff) {
	int_transfer_id++;
	reg_udral_physics_kinematics_rotation_Planar_0_1 js_msg =
	{
			.angular_position = pos,
			.angular_velocity = vel,
			.angular_acceleration = eff
	};
    cy_interface->send_msg<JS_msg>(
		&js_msg,
		js_sub_port_id[node_id-1],
		&int_transfer_id
	);
}


float qw = 0.0;
float qx = 0.0;
float qy = 0.0;
float qz = 0.0;
float avx = 0.0;
float avy = 0.0;
float avz = 0.0;
float lax = 0.0;
float lay = 0.0;
float laz = 0.0;

class IMUReader: public AbstractSubscription<State> {
public:
    IMUReader(InterfacePtr interface): AbstractSubscription<State>(interface,
        // Тут параметры - port_id, transfer kind или только port_id
        AGENT_IMU_PORT
    ) {};
    void handler(const reg_udral_physics_kinematics_cartesian_State_0_1& IMU_read, CanardRxTransfer* transfer) override {
        //std::cout << "Node id: " << +transfer->metadata.remote_node_id << std::endl;
        qw = IMU_read.pose.orientation.wxyz[0];
        qx = IMU_read.pose.orientation.wxyz[1];
        qy = IMU_read.pose.orientation.wxyz[2];
        qz = IMU_read.pose.orientation.wxyz[3];

        lax = IMU_read.twist.linear.meter_per_second[0];
        lay = IMU_read.twist.linear.meter_per_second[1];
        laz = IMU_read.twist.linear.meter_per_second[2];

        avx = IMU_read.twist.angular.radian_per_second[0];
        avy = IMU_read.twist.angular.radian_per_second[1];
        avz = IMU_read.twist.angular.radian_per_second[2];

    }
};
IMUReader * IMU_reader;

class HBeatReader: public AbstractSubscription<HBeat> {
public:
    HBeatReader(InterfacePtr interface): AbstractSubscription<HBeat>(interface,
        // Тут параметры - port_id, transfer kind или только port_id
        uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_
    ) {};
    void handler(const uavcan_node_Heartbeat_1_0& hbeat, CanardRxTransfer* transfer) override {
        std::cout << +transfer->metadata.remote_node_id << ": " << hbeat.uptime <<  std::endl;
    }
};
HBeatReader * reader;


namespace ruka
{

hardware_interface::CallbackReturn RukaSensor::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  //hw_sensor_change_ = stod(info_.hardware_parameters["example_param_max_sensor_change"]);
  // // // END: This part here is for exemplary purposes - Please do not copy to your production code 

  hw_sensor_states_.resize(
    info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  std::cout<<"info_.sensors[0].state_interfaces.size() "<<info_.sensors[0].state_interfaces.size()<<std::endl;
  std::cout<<"info_.sensors[0].name "<<info_.sensors[0].name<<std::endl;

    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  {
       std::cout<<"info_.sensors[0].state_interfaces[i].name "<<info_.sensors[0].state_interfaces[i].name<<std::endl;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RukaSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // export sensor state interface
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
  }

  return state_interfaces;
}

hardware_interface::CallbackReturn RukaSensor::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RukaSensor"), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RukaSensor"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RukaSensor"), "Successfully activated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RukaSensor::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("RukaSensor"), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RukaSensor"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  RCLCPP_INFO( 
    rclcpp::get_logger("RukaSensor"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RukaSensor::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{


  for (uint i = 0; i < hw_sensor_states_.size(); i++)
  {
    // Simulate RRBot's sensor data
    unsigned int seed = time(NULL) + i;

    hw_sensor_states_[0] = qx;  //orientation_x_
    hw_sensor_states_[1] = qy;  //orientation_y_
    hw_sensor_states_[2] = qz;  //orientation_z_ 
    hw_sensor_states_[3] = qw;  //orientation_w_

    hw_sensor_states_[4] = avx;  //angular_velocity_x_
    hw_sensor_states_[5] = avy;  //angular_velocity_y_
    hw_sensor_states_[6] = avz;  //angular_velocity_z_

    hw_sensor_states_[7] = lax;  //linear_acceleration_x_
    hw_sensor_states_[8] = lay;  //linear_acceleration_y_
    hw_sensor_states_[9] = laz;  //linear_acceleration_z_

      // RCLCPP_INFO(
      // rclcpp::get_logger("RukaSensor"), "Got state %f for sensor %u!",
      // hw_sensor_states_[i], i);
  }
  return hardware_interface::return_type::OK;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CallbackReturn RukaSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  cy_interface = CyphalInterface::create_heap<LinuxCAN, O1Allocator>(100, "can0", 1000, utilities); //Node ID, transport, queue_len, utilities
  reader = new HBeatReader(cy_interface);
  JS_reader_01 = new JSReader_01(cy_interface);
  IMU_reader = new IMUReader(cy_interface);
  RegAccessReader = new RegisterAccessReader(cy_interface);

  // robot has 6 joints and 2 interfaces
  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_position_command_.assign(6, 0);
  joint_velocities_command_.assign(6, 0);

  // force sensor has 6 readings
  ft_states_.assign(6, 0);
  ft_command_.assign(6, 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> RukaSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    //std::cout<<state_interfaces[ind].get_name()<<" :pos"<<std::endl;
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RukaSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  command_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_command_[0]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_command_[1]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_command_[2]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_command_[3]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_command_[4]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_command_[5]);

  return command_interfaces;
}

return_type RukaSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
  // {
  //   joint_velocities_[i] = joint_velocities_command_[i];
  //   joint_position_[i] += joint_velocities_command_[i] * period.seconds();
  // }

  for (auto i = 0ul; i < joint_position_.size(); i++)
  {
    joint_position_[i] = j_pos[i];
    joint_velocities_[i] = j_vel[i];
  }
  cy_interface->loop();
  return return_type::OK;
}

int itera = 0;
return_type RukaSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
float eff = 0; //TODO get Efforrt from ROS2_Control

for (auto i = 0ul; i < joint_velocities_command_.size() ; i++) //
{
send_JS(i+1, (float)joint_position_command_[i], (float)joint_velocities_command_[i], (float)eff);
std::cout<<"jn: "<<i<<" pos: "<<joint_position_command_[i]<<std::endl;
cy_interface->loop();
}

 if (itera > 1000)
 {
    heartbeat();
    //std::cout<<"HB sent"<<std::endl;
    itera = 0;
  }
  cy_interface->loop();
  itera++;
  return return_type::OK;
}

}  // namespace ros2_control_demo_example_7

#include "pluginlib/class_list_macros.hpp"


PLUGINLIB_EXPORT_CLASS(
  ruka::RukaSystem, hardware_interface::SystemInterface)

PLUGINLIB_EXPORT_CLASS(
  ruka::RukaSensor,
  hardware_interface::SensorInterface)