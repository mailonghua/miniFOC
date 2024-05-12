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
  // Serial.begin(115200);
  // Serial1.begin(115200, SERIAL_8N1, 18, 17);
  SERIAL_INIT(115200);
  // while (1)
  // {
  //   Serial.println("This is serial0....");
  //   Serial1.println("This is serial1....");
  //   sleep(2);
  //   INFO("This is info test\n");
  //   ERR("This is err test");
  // }
  const char *version = FIRMWARE_VERSION;
  INFO("Motor System(%s) start...\n", version);
  HAL::HAL_Init();
  Start_UserTask();
}

void loop()
{
  HAL::Motor_Update(NULL);
  HAL::HAL_Update();
}