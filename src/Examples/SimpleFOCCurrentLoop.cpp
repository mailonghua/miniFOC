/*
#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
static BLDCMotor motor = BLDCMotor(7,7.8); 
static BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 7, 4);

// instantiate the commander
static Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

//电流检测
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.f, 8, 9);

void setup() {
  // monitoring port
  Serial.begin(115200); 
  //1.磁编码器设置 
  Wire.begin(1,0,(uint32_t)400000);
  Wire.setClock(400000);//400000
  // initialise magnetic sensor hardware
  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  //2.驱动器设置
  driver.voltage_power_supply = 24;
  driver.init();

  current_sense.linkDriver(&driver);
  // 连接电机和驱动器
  motor.linkDriver(&driver);

  motor.torque_controller = TorqueControlType::foc_current;//foc电流力矩控制
  motor.controller = MotionControlType::torque;
  // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // motor.torque_controller = TorqueControlType::voltage;
  // motor.controller = MotionControlType::torque;
  //电流环的q和d的PID
  motor.PID_current_q.P = 5;
  motor.PID_current_q.I= 200;
  motor.PID_current_d.P= 5;
  motor.PID_current_d.I = 200;

  //低通滤波器
  motor.LPF_current_q.Tf = 0.01; 
  motor.LPF_current_d.Tf = 0.01; 

   // 校准电压
  //motor.voltage_sensor_align = 3;
  motor.current_limit = 1.5;

  motor.useMonitoring(Serial);
   // 初始化电机
  motor.init();

    //3.电流检测传感器
  Serial.printf("%d,%d\n",analogRead(8),analogRead(9));
  // 初始化电流检测
  if (current_sense.init())  Serial.println("Current sense init success!");
  else{
    Serial.println("Current sense init failed!");
    return;
  }
  motor.linkCurrentSense(&current_sense);
  // 校准编码器，启用FOC
  motor.initFOC();

  //设置电机初目标

  //command.add('T', doTarget, "target velocity");
  command.add('T', doTarget, "target current");
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