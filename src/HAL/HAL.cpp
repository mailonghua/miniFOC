#include "HAL/HAL.h"

void HAL::HAL_Init()
{
    OTA_Init();
}

void HAL::HAL_Update()
{
    OTA_Update();
    //Motor_Test();
}