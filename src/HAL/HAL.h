#ifndef _HAL__H_
#define _HAL__H_
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <SimpleFOC.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <map>
#include "driver/gpio.h"
#include "driver/twai.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "Config.h"
#include "HAL_Def.h"
#include "CuteBuzzerSounds.h" //https://github.com/GypsyRobot/CuteBuzzerSounds
#include "HAL_Timer.h"//include Log.h

// DEBUG_SET_LEVEL(DEBUG_LEVEL_DEBUG);
// extern std::string Current_IP_Address;
/*线程等待一定时间,按照一定频率执行线程while(1)*/
#define INIT_TASK_TIME(WAIT_MS)                             \
    portTickType xLastWakeTime;                             \
    const portTickType xFrequency = pdMS_TO_TICKS(WAIT_MS); \
    xLastWakeTime = xTaskGetTickCount();

#define WAIT_TASK_TIME() \
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
enum FOC_TYPE
{
    OPEN_LOOP = 0,
};
enum STATUS_RETURN
{
    ERROR = -1,
    SUCCESS = 0
};
namespace HAL
{
    void HAL_Init();
    void HAL_Update();

    // OTA
    void OTA_Init();
    void OTA_Update();

    // Motor-simplefoc
    void Motor_Init();
    void Motor_Update(void *parameter);

    // NVS-EEPROM
    int NVS_Init();
    void NVS_End();
    bool NVC_Clear();
    float put_float(const char *key, float_t value);
    float get_float(const char *key, float_t defaultValue = NAN);
    int put_int(const char *key, int32_t value);
    int get_int(const char *key, int32_t defaultValue = 0);

    // CAN
    void CAN_Init();
    void CAN_Update(void *);
    void CAN_Receive(ReceiveMotorData *recv_data);
    void CAN_Send(SendMotorData &send_data, uint32_t MsgID);
    void CAN_Receive_Template();
    void CAN_Send_Template();
    ReceiveMotorData *CAN_Get_Data();
    uint8_t CAN_GetCurrentMotorID();
    void CAN_SetCurrentMotorID(uint8_t id);

}
#endif