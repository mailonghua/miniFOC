/*
#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
static BLDCMotor motor = BLDCMotor(7); 
static BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 7, 4);

// 设置目标电压
float target_voltage = 2;
// instantiate the commander
static Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

//电流检测
// 电流检测
//InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, 8, 9);

void setup() {
  // monitoring port
  Serial.begin(115200);

  // configure i2C
  
  Wire.begin(1,0,(uint32_t)400000);
  Wire.setClock(400000);//400000
  // initialise magnetic sensor hardware
  sensor.init(&Wire);
  motor.linkSensor(&sensor);//1.链接传感器和电机
  //驱动器
  driver.voltage_power_supply = 24;
  driver.init();
  // 连接电机和驱动器
  motor.linkDriver(&driver);//2.链接电机驱动和电机
  motor.voltage_limit = 10;   // Volts 【limiting motor current】
  //motor.velocity_limit = 5; // [rad/s] cca 50rpm
  //motor.controller = MotionControlType::velocity_openloop;
   // 校准电压
  motor.voltage_sensor_align = 3;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.useMonitoring(Serial);
   // 初始化电机
  motor.init();
  // 校准编码器，启用FOC
  motor.initFOC();

  //设置电机初目标

  command.add('T', doTarget, "target velocity");

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  motor.loopFOC();
  // 开环速度运动
  // 使用电机电压限制和电机速度限制
  motor.move();
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  command.run();
  //sensor.update();
  
  // display the angle and the angular velocity to the terminal

  // Serial.print(sensor.getAngle());
  // Serial.print("\t");
  // Serial.println(sensor.getVelocity());
}
*/