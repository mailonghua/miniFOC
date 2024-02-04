#include "HAL/HAL.h"

void HAL::HAL_Init()
{
    OTA_Init();
    Motor_Init();
    cute.init(BUZZER_PIN);//声音初始化
    cute.play(S_CONNECTION);//播放链接声音
}

void HAL::HAL_Update()
{
    OTA_Update();
    //Motor_Test();
}