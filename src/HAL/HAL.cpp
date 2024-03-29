#include "HAL/HAL.h"

void HAL::HAL_Init()
{
    // OTA_Init();
    NVS_Init();
    CAN_Init();
    Motor_Init();
    cute.init(BUZZER_PIN); // 声音初始化-相关仿真https://wokwi.com/projects/376201088002318337
    cute.play(S_START);    // 播放链接声音
    NVS_End();
#if UART_RECEIVE_SELF_METHORD == 1
    Uart_Receive_IRQ_Register();
#endif
}

void HAL::HAL_Update()
{
    // OTA_Update();
    //  Motor_Test();
}