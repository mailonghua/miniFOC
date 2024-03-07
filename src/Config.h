#ifndef __CONFIG_H__
#define __CONFIG_H__
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

/*
    Define VOFA trasnport type
    0:UART
    1:UDP
    2:TCP[not use]
*/
#define VOFA_TRANSMISSION_TYPE (1)
#define VOFA_UDP_LOCAL_PORT 2333
#define VOFA_UDP_REMOTE_PORT 2334

// 控制用simpleFOC Studio的串口输出控制
#define SIMPLEFOC_DEBUG_ENABLE 0
#endif