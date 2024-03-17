#if 0
#include <SimpleFOC.h>
static MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);
static BLDCMotor motor = BLDCMotor(7);
static BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 7, 4);
static InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.f, 13, 14);
static Commander command = Commander(Serial);
static void doTarget(char *cmd) { command.scalar(&motor.target, cmd); }
static void doMotor(char *cmd) { command.motor(&motor, cmd); }
void setup()
{

  Serial.begin(115200);
  Serial.println("Hello word");
  Wire.begin(1, 0, (uint32_t)400000);
  Wire.setClock(400000); // 400000
  // initialise magnetic as5600 hardware
  as5600.init(&Wire);
  motor.linkSensor(&as5600);
  driver.voltage_power_supply = 24;
  driver.init();
  current_sense.linkDriver(&driver);
  // 连接电机和驱动器
  motor.linkDriver(&driver);
  motor.torque_controller = TorqueControlType::foc_current; // foc电流力矩控制
  motor.controller = MotionControlType::torque;
  // 电流环的q和d的PID
  motor.PID_current_q.P = 30; // 5;
  motor.PID_current_q.I = 25; // 300;
  motor.PID_current_q.D = 1;  // 添加了D后，打开后电机,在打开串口条件下，启动会有转动
  motor.PID_current_d.P = 18; // 5;
  motor.PID_current_d.I = 30; // 300;
  motor.PID_current_d.D = 1;  // 打开后电机启动，在打开串口条件下，会有异常转动
  //  低通滤波器
  motor.LPF_current_q.Tf = 0.5;
  motor.LPF_current_d.Tf = 0.5;
  motor.current_limit = 1.0;
  motor.useMonitoring(Serial);
  // 初始化电机
  motor.init();
  current_sense.init();
  current_sense.gain_b *= -1;
  motor.linkCurrentSense(&current_sense);
  motor.initFOC();

  motor.target = 0.0f;
  command.add('T', doTarget, "target current");
  command.add('M', doMotor, "my motor");
  _delay(1000);
}
void loop()
{
  // motor.loopFOC();
  // motor.move();
  // motor.monitor();
  // command.run();
  sleep(10);
}
#else
#include "HAL/HAL.h"
void Start_UserTask()
{
  // 1.启动FOC算法-循环频率
  xTaskCreate(HAL::CAN_Update, "CAN_Updata", 4096, NULL, 0, NULL);
  HAL::StartSystemStateDetectTask();
  HAL::StartInputKetDetectTask();
  vTaskDelay(pdMS_TO_TICKS(1000));
  INFO("The number of threads currently startd is %d\n", uxTaskGetNumberOfTasks());
}
void setup()
{
  Serial.begin(115200);
  HAL::HAL_Init();
  Start_UserTask();
}

void loop()
{
  HAL::Motor_Update(NULL);
  HAL::HAL_Update();
}
#endif