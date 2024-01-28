#ifndef _HAL__H_
#define _HAL__H_
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h> 
#include <SimpleFOC.h>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "Config.h"
#include "HAL_Def.h"
#include "Log.h"


DEBUG_SET_LEVEL(DEBUG_LEVEL_DEBUG);
extern String Current_IP_Address;
/*线程等待一定时间,按照一定频率执行线程while(1)*/
#define INIT_TASK_TIME(WAIT_MS) \
        portTickType xLastWakeTime; \
        const portTickType xFrequency = pdMS_TO_TICKS(WAIT_MS); \
        xLastWakeTime = xTaskGetTickCount();   \

#define WAIT_TASK_TIME() \
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

namespace HAL
{
    void HAL_Init();
    void HAL_Update();


    //OTA
    void OTA_Init();
    void OTA_Update();

    //Motor-simplefoc
    void Motor_Init();
    void Motor_Update();

}
#endif