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

#define REBOOT_SYS()            \
    cute.play(S_DISCONNECTION); \
    sleep(2);                   \
    ESP.restart();

#define INIT_CAN_MSG(message)                           \
    message.identifier = MOTOR_CAN_ID + currentMotorID; \
    message.extd = 0;                                   \
    message.rtr = 0;                                    \
    message.self = 0;                                   \
    message.data_length_code = 8;

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
    WIFI_SELECT_MODE, // 正常链接的状态音
    WIFI_NO_CONNECT,  // 没有链接的状态音
    WIFI_DISCONNECT,  // 正常断开连接的状态音
    SYS_INTERNAL_ERROR,
};
// UART Command define
enum UART_RECEIVE_COMMAND
{

    GET_SYS_STATE_CMD = 0x5500,          // 获取系统状态
    OPEN_WIFI_PWR_CMD = 0x5501,          // 开启WIFI
    CLOSE_WIFI_PWR_CMD = 0x5502,         // 关闭WIFI
    CLEAR_NVS_CMD = 0x5503,              // 清空NVS
    OPEN_BLE_PWR_CMD = 0x5504,           // 开启BLE
    CLOSE_BLE_PWR_CMD = 0x5505,          // 关闭BLE
    SET_WIFI_SSID_PASSWD = 0x5506,       // 设置重新修改WIFI和SSID
    DISABLE_MOTOR_DISABLE = 0x5507,      // 失能电机
    CHOOSE_MOTOR_CURRENT_MODE = 0x5508,  // 力矩模式
    CHOOSE_MOTOR_VELOCITY_MODE = 0x5509, // 速度模式
    CHOOSE_MOTOR_POSITION_MODE = 0x5510, // 位置模式
    OPEN_MOTOR_WAVAE_DEBUG = 0x5511,     // 开启波形输出，适用VOFA格式
    SET_MOTOR_TARGET = 0x5454,           // 0x5454:对应的ASCII是TT 设置当前电机的目标 0x5511 + 2Byte(电机的目标(只能设置整数))
    SET_MOTOR_CAN_ID = 0x4944,           // 设置电机的CAN ID,指令对应的字符串：ID
    SET_SYSTEM_REBOOT = 0x5513,          // 让电机系统重启

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
    void StartSystemStateDetectTask();          // 线程调用的函数
    void changeShowSysMode(SYS_STATE_SELECT);   // 改变LED工作的状态
    // OTA
    bool OTA_Init();
    void OTA_Update();
    void OTA_SwitchStatus();

    // Motor-simplefoc
    void Motor_Init();
    void Motor_Update(void *parameter);
    void Motor_GetCurrentState(MotorFeedData &data);
    void Motor_GetCurrentState(MOTOR_RawData_t &data);
    bool Motor_Disable();
    void Motor_ZeroTarget();
    void Motor_SetTarget(float data);
    int Get_MotorState();
    void Motor_SwitchMode(UART_RECEIVE_COMMAND mode);

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
    void StartInputKetDetectTask();
    void Uart_Receive_IRQ_Register();
    UART_RECEIVE_COMMAND Get_CurrentMotorStatus();
}
#endif