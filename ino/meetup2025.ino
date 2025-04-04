/**
 *
 * Velocity motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target velocity (in radians per second) from cyphal
 *
 *
 * By using the serial terminal set the velocity value you want to motor to obtain
 *
 */
#include <SimpleFOC.h>
#include <VBCoreG4_arduino_system.h>

#include <utility>
#include <cyphal.h>

#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/node/Health_1_0.h>
#include <uavcan/node/Mode_1_0.h>
#include <uavcan/si/unit/angular_velocity/Scalar_1_0.h>
#include "reg/udral/physics/kinematics/rotation/Planar_0_1.h"
#include <uavcan/si/unit/angle/Scalar_1_0.h>
#include <uavcan/primitive/array/Real16_1_0.h>
#include <reg/udral/physics/kinematics/cartesian/Twist_0_1.h>

// magnetic sensor instance - SPI
SPIClass SPI_3(PC12, PC11, PC10);
MagneticSensorSPI sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);

BLDCMotor motor = BLDCMotor(15);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10);

InlineCurrentSense current_sense = InlineCurrentSense(45.0, PC1, PC2, PC3); 


template <class T>
class ReservedObject {
private:
    unsigned char buffer[sizeof(T)];
    T* obj;
public:
    template <class... Args>
    void create(Args&&... args) {
        obj = new (buffer) T(std::forward<Args>(args)...);
    }

    T* pointer() {
        return obj;
    }
    T* operator->() {
        return obj;
    }

    ~ReservedObject() {
        obj->~T();
    }
};

TYPE_ALIAS(Odom, reg_udral_physics_kinematics_rotation_Planar_0_1)
TYPE_ALIAS(Twist, reg_udral_physics_kinematics_cartesian_Twist_0_1)
TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(ArrayK, uavcan_primitive_array_Real16_1_0)

static uint8_t CYPHAL_HEALTH_STATUS = uavcan_node_Health_1_0_NOMINAL;
static uint8_t CYPHAL_MODE = uavcan_node_Mode_1_0_INITIALIZATION;

CanFD* canfd;
FDCAN_HandleTypeDef* hfdcan1;
static bool _is_cyphal_on = false;
static std::shared_ptr<CyphalInterface> cyphal_interface;

typedef uint32_t millis_t;
typedef uint64_t micros_t;

void heartbeat();
void send_odom_data();

void error_handler() {Serial.println("error"); while (1) {};}
UtilityConfig utilities(micros, error_handler);

constexpr micros_t MICROS_S = 1'000'000;


static constexpr CanardNodeID NODE_ID = 0;

static constexpr CanardPortID ODOM_PORT = (NODE_ID * 100) + 2000; //2000
static constexpr CanardPortID K_PORT = ODOM_PORT + 10;//2010
static constexpr CanardPortID CMD_VEL_SUB = ODOM_PORT + 50; //2050


static millis_t uptime = 0;
millis_t t_heartbeat;
millis_t t_vel;

float target_velocity=0;
float kp = 0.2f, ki = 5, kd = 0;

class CmdVelSub: public AbstractSubscription<Twist> {
public:
    CmdVelSub(InterfacePtr interface, CanardPortID port_id):
          AbstractSubscription<Twist>(interface, port_id)
          {};
    void handler(const Twist::Type& msg, CanardRxTransfer* _) override {
        target_velocity = msg.angular.radian_per_second[2];
        Serial.println(target_velocity);
    }
};

class KSub: public AbstractSubscription<ArrayK>{
public:
    KSub(InterfacePtr interface, CanardPortID port_id):
          AbstractSubscription<ArrayK>(interface, port_id)
          {};
    void handler(const ArrayK::Type& msg, CanardRxTransfer* _) override {
        kp = msg.value.elements[0];
        ki = msg.value.elements[1];
        kd = msg.value.elements[2];
        motor.PID_velocity.P = kp;
        motor.PID_velocity.I = ki;
        motor.PID_velocity.D = kd;
        delay(100);
        Serial.println(motor.PID_velocity.P);
        Serial.println(motor.PID_velocity.I);
        Serial.println(motor.PID_velocity.D);
    }
};


ReservedObject<CmdVelSub> cmd_vel_sub;
ReservedObject<KSub> k_sub;

float offset_angle;
void setup() {

  pinMode(PB5, INPUT);
  pinMode(PB3, OUTPUT);
  pinMode(PB15, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB13, OUTPUT);

  digitalWrite(PB15, HIGH);
  digitalWrite(PB14, HIGH);
  digitalWrite(PB13, HIGH);
  digitalWrite(PB3, HIGH);

  //init can fd
  SystemClock_Config();  // Настройка тактирования
  canfd = new CanFD();  // Создаем управляющий класс
  canfd->init(); // Инициализация тактирования CAN и пинов
  hfdcan1 = canfd->get_hfdcan(); //получаем ссылку на конкретный инстанс
  canfd->write_default_params();  // Записываем дефолтные параметры для FDCAN
  canfd->apply_config();  // Применяем их
  canfd->default_start();
 
  size_t queue_len = 200;//размер очереди сообщений
  cyphal_interface = std::shared_ptr<CyphalInterface>(CyphalInterface::create_heap<G4CAN, O1Allocator>(
        NODE_ID,
        hfdcan1,
        queue_len,
        utilities
    ));
  cmd_vel_sub.create(cyphal_interface, CMD_VEL_SUB);
  k_sub.create(cyphal_interface, K_PORT);
  _is_cyphal_on = true;
  CYPHAL_MODE = uavcan_node_Mode_1_0_OPERATIONAL;


  // initialise magnetic sensor hardware
  sensor.init(&SPI_3);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  driver.pwm_frequency = 25000;    // Частота ШИМ (в Гц)
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = kp;
  motor.PID_velocity.I = ki;
  motor.PID_velocity.D = kd;
  // default voltage_power_supply
  motor.voltage_limit = 24;

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  current_sense.linkDriver(&driver);
  current_sense.init();
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

  // use monitoring with serial
  Serial.begin(115200);

  // initialize motor
  motor.init();
  
  /* Инициализация FOC c калибровкой!!! 
  Закоментируйте  строки с current_sense.skip_align = ... до  motor.initFOC(); включительно
  Раскомментируйте  следующие 5 строк*/
  motor.initFOC();  
  Serial.print("Zero electric offset: ");
  Serial.println(motor.zero_electric_angle);
  Serial.print("Sensor direction: ");
  Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
  
  // current_sense.skip_align  = true;
  // motor.zero_electric_angle = 5.91; //3.23 -- big  5.91 -- small
  // motor.sensor_direction = Direction::CW; 
  // // !!! Инициализация FOC БЕЗ калибровки
  // motor.initFOC();

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using cyphal:"));
  offset_angle = sensor.getAngle();
  t_vel = t_heartbeat = millis();
  _delay(1000);
}

void loop() {

  cyphal_interface->loop();
  
  if(millis() - t_heartbeat >= 1000){
      heartbeat();
      digitalToggle(PD2);
      t_heartbeat = millis();
    }

  if(millis() - t_vel >= 20){
      send_odom_data();
      t_vel = millis();
    }

  motor.loopFOC();
  motor.move(target_velocity);

}

void heartbeat() {
    
    static CanardTransferID hbeat_transfer_id = 0;
    HBeat::Type heartbeat_msg = {
        .uptime = uptime,
        .health = {CYPHAL_HEALTH_STATUS},
        .mode = {CYPHAL_MODE}
    };
    uptime += 1;

    if (_is_cyphal_on) {
        cyphal_interface->send_msg<HBeat>(
            &heartbeat_msg,
            uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
            &hbeat_transfer_id,
            MICROS_S * 2
        );
            

    }
}

void send_odom_data(){
    static CanardTransferID odom_transfer_id = 0;

    Odom::Type msg = {
        .angular_position = sensor.getAngle() - offset_angle,
        .angular_velocity = motor.shaft_velocity
    };
 
    cyphal_interface->send_msg<Odom>(&msg, ODOM_PORT, &odom_transfer_id);
}

