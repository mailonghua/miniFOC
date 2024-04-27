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
/*FOC算法-电机控制线程函数*/
void Start_MotorTask(void *)
{
  INFO("MotorTask thread create success...\n");
  INIT_TASK_TIME(1);
  while (1)
  {
    HAL::Motor_Update(NULL);
    WAIT_TASK_TIME();
  }
}
/*系统监视线程*/
void Start_SysMonitoringThread(void *)
{
  INFO("Sys monitoring thread create success...\n");
  INIT_TASK_TIME(500); // 500ms
  while (true)
  {
    /*1.检查芯片温度--超过阈值直接停机*/
    float mcuTemp = HAL::MCU_Internal_Temperature_float();
    if (mcuTemp >= ESP32S3FH4R2_TMP)
    {
      HAL::Motor_Disable();
      // 蜂鸣器提醒
      for (int i = 0; i < 3; i++)
      {
        HAL::playSysMusic(SYS_INTERNAL_ERROR);
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      ERR("The current chip temperature is too high(%f) and the motor disable...\n", mcuTemp);
    }
    // 根据模式进行判断
    switch (HAL::Get_CurrentMotorStatus())
    {
    case CHOOSE_MOTOR_CURRENT_MODE: // 电流模式
    {
      {
        // a.如果当前的目标值为0，但是速度和力矩在一定时间不为0，则报错
        HAL::MOTOR_RawData_t data;
        HAL::Motor_GetCurrentState(data);
        if ((data.target == 0) && (data.toque_q > 0.2))
        {
          vTaskDelay(pdMS_TO_TICKS(4000));
          if ((data.target == 0) && (data.toque_q > 0.2))
          {
            HAL::Motor_Disable();
            // 蜂鸣器提醒
            for (int i = 0; i < 3; i++)
            {
              HAL::playSysMusic(SYS_INTERNAL_ERROR);
              vTaskDelay(pdMS_TO_TICKS(1000));
            }
            ERR("The motor rotates abnormally(%f,%f), turn off the motor...\n", data.target, data.toque_q);
            REBOOT_SYS();
          }
        }
      }
      break;
    }
    case CHOOSE_MOTOR_VELOCITY_MODE:
      break;
    case CHOOSE_MOTOR_POSITION_MODE:
      break;
    default:
      break;
    }
    /*2.检查目标值和*/
    /*.检查CAN控制器的接收频率--若是低于某个频率则报警(可能丢帧)*/
    WAIT_TASK_TIME();
  }
}
void Start_UserTask()
{
  // 1.启动FOC算法-循环频率
  xTaskCreate(HAL::CAN_Update, "CAN_Updata", 4096, NULL, 0, NULL);
  HAL::StartSystemStateDetectTask();
  HAL::StartInputKetDetectTask();
  vTaskDelay(pdMS_TO_TICKS(1000));
  // xTaskCreate(Start_MotorTask, "MotorUpdate", 4096, NULL, 1, NULL);               // 当前阶段最高优先级
  xTaskCreate(Start_SysMonitoringThread, "SysStatusDetect", 4096, NULL, 0, NULL); // 系统检测线程
  INFO("The number of threads currently startd is %d\n", uxTaskGetNumberOfTasks());
}

void setup()
{
  Serial.begin(115200);
  INFO("Motor System start...\n");
  HAL::HAL_Init();
  Start_UserTask();
}

void loop()
{
  HAL::Motor_Update(NULL);
  HAL::HAL_Update();
}
#endif