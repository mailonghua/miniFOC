#ifndef __CONFIG_H__
#define __CONFIG_H__
#include "version.h"
// 定义串口控制的UART --用于决定串口输入控制使用那个串口，关于串口的初始化定义在Log.h文件中
#define UART_RECV_CTL() Serial
// 系统检测线程阈值
#define ESP32S3FH4R2_TMP 90
// 控制用simpleFOC Studio的串口输出控制
#define SIMPLEFOC_DEBUG_ENABLE 0

// 串口接收选择
// 1：使用自定义的串口的指令
// 0：使用SimpleFOC的指令处理
#define UART_RECEIVE_SELF_METHORD 1

// I2C
#define I2C_SDA_0 GPIO_NUM_1
#define I2C_SCL_0 GPIO_NUM_0

// MOTOR
#define MOTOR_A GPIO_NUM_5
#define MOTOR_B GPIO_NUM_6
#define MOTOR_C GPIO_NUM_7
#define CURRENT_SENSOR_A GPIO_NUM_8
#define CURRENT_SENSOR_B GPIO_NUM_9

// CAN
#define CAN_TX GPIO_NUM_10
#define CAN_RX GPIO_NUM_11
#define MOTOR_CAN_ID (0x300)
#define MOTOR_MAX_CURRENT_IQ (900) // ma

// INPUT
#define KEY GPIO_NUM_21
#define KEY_DETECT_THREAD_DELAY (50)
#define SELECT_CAN_ID_DELAY_MAX (10) // 20*KEY_DETECT_THREAD_DELAY = 1000ms，小于1秒的按压都是选择CAN
#define SELECT_OPEN_WIFI_MIN (20)    //
#define SELECT_CLEAR_NVS_MIN (50)    //
// buzzer
#define BUZZER_PIN 20
#define STATUS_LED 19

#define EEPROM_MAX_SIZE 128
#define STASSID "Xiaomi_Mailonghua"
#define STAPSK "mlh8823727"
// #define STASSID "OPPO Find X7 Ultra"
// #define STAPSK "123456789"
/*
    Define VOFA trasnport type
    0:UART
    1:UDP
    2:TCP[not use]
*/
#define VOFA_TRANSMISSION_TYPE (1)
#define VOFA_UDP_LOCAL_PORT 2333
#define VOFA_UDP_REMOTE_PORT 2334

#endif