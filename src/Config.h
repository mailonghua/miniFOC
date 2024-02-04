#ifndef __CONFIG_H__
#define __CONFIG_H__
//I2C
#define I2C_SDA_0 GPIO_NUM_1
#define I2C_SCL_0 GPIO_NUM_0

//MOTOR
#define MOTOR_A  GPIO_NUM_5
#define MOTOR_B  GPIO_NUM_6
#define MOTOR_C  GPIO_NUM_7
#define CURRENT_SENSOR_A GPIO_NUM_8
#define CURRENT_SENSOR_B GPIO_NUM_9

//CAN
#define CAN_TX GPIO_NUM_10
#define CAN_RX GPIO_NUM_11

//buzzer
#define BUZZER_PIN 10

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
#endif