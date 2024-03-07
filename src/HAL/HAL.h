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
#include "HAL_Timer.h"        //include Log.h
#include "esp32-hal.h"        // 引入ESP32 HAL库
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
enum SYS_STATE
{
    SYS_NORMAL = 0,
    SYS_ERROR,
    SYS_FATAL,
};
// 系统状态的选择
enum SYS_STATE_SELECT
{
    SYS_START = 0,
    CAN_ID_SELECT_MODE,
    WIFI_SELECT_MODE,
    SYS_INTERNAL_ERROR,
};
namespace HAL
{
    void HAL_Init();
    void HAL_Update();
    // 内部环境函数
    float MCU_Internal_Temperature_float();
    uint8_t MCU_Internal_Temperature_u8();
    void setSysState(SYS_STATE status);
    uint8_t getSysState();
    void playSysMusic(SYS_STATE_SELECT select); // 播放音乐的选择
    void selectSysLed(SYS_STATE_SELECT select); // 选择系统状态灯
    void systemStateUpdate();                   // 线程调用的函数
    void changeLedMode(SYS_STATE_SELECT);       // 改变LED工作的状态
    // OTA
    void OTA_Init();
    void OTA_Update();
    void OTA_SwitchStatus();

    // Motor-simplefoc
    void Motor_Init();
    void Motor_Update(void *parameter);
    void Motor_GetCurrentState(MotorFeedData &data);

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
    uint8_t &CAN_GetCurrentMotorID();
    void CAN_SetCurrentMotorID(uint8_t id);

    // 输入的外设控制
    void Input_Key_Init();
    void Uart_Receive_Init();

}
#endif