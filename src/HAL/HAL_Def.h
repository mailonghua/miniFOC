#ifndef __HAL_DEF__H_
#define __HAL_DEF__H_

#include <stdint.h>
namespace HAL
{
  /*Beep*/
  typedef enum beep_mode
  {
    CONVERT_MODE = 0,

    MAX_BEEP_MODE
  } BEEP_MODE;
  typedef struct
  {
    int16_t Motor_Current;
  } CAN_Send_t;

  typedef struct
  {
    uint16_t Rotor_Machinery_Angle; // 转子机械角度
    int16_t Rotor_Speed;            // 转子速度
    uint16_t Toque;                 // 转矩
    uint8_t tempetature;            // 电机温度
    uint8_t state;                  // 预留
  } CAN_Recv_t;
  // 电机的反馈状态
  typedef struct
  {
    float target;
    float shaft_angle;      // 累加角度值
    float electrical_angle; // 机械角度
    float speed;
    float toque_q;     // 力矩
    float toque_d;     // 力矩
    float tempetature; // 温度
  } MOTOR_RawData_t;
  typedef union
  {
    uint8_t data[8];
    CAN_Recv_t Motor_Data;
  } MotorFeedData;
  typedef struct
  {
    uint32_t Motor_CAN_ID;
    MotorFeedData Feed_Data;
  } SendMotorData; // 电机反馈数据

  typedef union
  {
    uint8_t data[2];
    CAN_Send_t Motor_Data;
  } ReceiveMotorData; // 电机接收数据

  typedef enum
  {
    KEY_ID_NONE = 0,
    KEY_ID_START = 1, // start
    KEY_ID_PREV = 1,
    KEY_ID_OK = 2,
    KEY_ID_NEXT = 3,
    KEY_ID_END = 3, // end
  } key_id_t;
  typedef struct
  {
    bool cur;
    bool last;
  } key_state_t;

// number of elements in an array
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
// byte offset of member in structure
#define MEMBER_OFFSET(structure, member) ((int)&(((structure *)0)->member))
// size of a member of a structure
#define MEMBER_SIZE(structure, member) (sizeof(((structure *)0)->member))

#define INLINE __inline__ __attribute__((always_inline))

#define assert_param(expr) ((void)0U)
  // #define MOTOR1_CAN_ID (0x200)

}
#endif