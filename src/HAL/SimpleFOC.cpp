
#include "HAL.h"
#include <SimpleFOC.h>
#include <algorithm>
#include "esp32-hal.h" // 引入ESP32 HAL库

static MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);
static BLDCMotor motor = BLDCMotor(7);
static BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 7, 4);
static InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.f, 13, 14);
static Commander command = Commander(Serial);
// 用于存储simplefoc初始化状态,如初始化失败则停止foc的算法执行，并disable motor
static int simpleFOCInitFlag = 0; //: 0：代表正常1：代表异常，用于循环检测的判断异常

void doTarget(char *cmd) { command.scalar(&motor.target, cmd); }
void doMotor(char *cmd) { command.motor(&motor, cmd); }
#define MOTO_DIRECTION_PARA "motorDirection"
#define MOTO_ANGLE_PARA "motorAngle"
extern QueueHandle_t Motor_Queue; // can文件中定义的

void encode_init()
{ // 编码器初始化
  // 1.磁编码器设置
  Wire.begin(1, 0, (uint32_t)400000);
  Wire.setClock(400000); // 400000
  // initialise magnetic as5600 hardware
  as5600.init(&Wire);
  motor.linkSensor(&as5600);
}
// 驱动器初始化
void driver_init()
{
  driver.voltage_power_supply = 24;
  driver.init();
}
// 电流传感器初始化
int current_sense_init()
{
  if (current_sense.init())
  {
    Serial.println("Current sense init success!");
  }
  else
  {
    Serial.println("Current sense init failed!");
    return 1;
  }
  // 结合硬件，可以看到当前的版本的B相的检测电阻和检测引脚方向是反的，因此这里取反
  current_sense.gain_b *= -1;
  return 0;
}

// 平滑调整电流的函数
// current: 当前电流值
// target: 目标电流值
// maxStep: 单次调整的最大步进值
// 返回调整后的电流值
//[当前暂时没有使用]
float smoothInterpolation(float current, float target, float maxStep)
{
  // 计算当前电流与目标电流之间的差值
  float difference = target - current;

  // 如果差值的绝对值小于等于最大步进值，直接返回目标电流，无需进一步插值
  if (std::abs(difference) <= maxStep)
  {
    return target;
  }

  // 如果差值大于0，说明需要增加电流，但增加的幅度不超过maxStep
  if (difference > 0)
  {
    return current + std::min(difference, maxStep);
  }

  // 如果差值小于0，说明需要减少电流，但减少的幅度不超过maxStep
  return current - std::min(-difference, maxStep);
}
// 获取CAN目标值
static void update_receive_can_target()
{
  HAL::ReceiveMotorData motor_target;
  if (pdTRUE == xQueueReceive(Motor_Queue, &motor_target, 0))
  {
    // 电流力矩模式
    if ((motor.torque_controller == TorqueControlType::foc_current) &&
        (motor.controller == MotionControlType::torque))
    {
      float recv_can_target = motor_target.Motor_Data.Motor_Current;
      if ((recv_can_target <= MOTOR_MAX_CURRENT_IQ) && (recv_can_target > -MOTOR_MAX_CURRENT_IQ))
      {
        INFO("Recv CAN target current :%f\n", recv_can_target / 1000.0);
        motor.target = recv_can_target / 1000.0;
      }
      else
      {
        ERR("Recv CAN target current :%f\n", recv_can_target / 1000.0);
      }
    }
  }
}
// FOC电流环
void foc_current_init(int direction = -1, float angle = -1)
{
  encode_init(); // 编码器初始化
  driver_init(); // 驱动初始化
  current_sense.linkDriver(&driver);
  // 连接电机和驱动器
  motor.linkDriver(&driver);
  motor.torque_controller = TorqueControlType::foc_current; // foc电流力矩控制
  motor.controller = MotionControlType::torque;
  // 电流环的q和d的PID
  motor.PID_current_q.P = 30; // 5;
  motor.PID_current_q.I = 25; // 300;
  motor.PID_current_q.D = 1.5;  // 添加了D后，打开后电机,在打开串口条件下，启动会有转动
  motor.PID_current_d.P = 18; // 5;
  motor.PID_current_d.I = 30; // 300;
  motor.PID_current_d.D = 1;  // 打开后电机启动，在打开串口条件下，会有异常转动
  //  低通滤波器
  motor.LPF_current_q.Tf = 0.5;
  motor.LPF_current_d.Tf = 0.5;
  motor.current_limit = 1.0;
  motor.useMonitoring(Serial);
  // 初始化电机
  motor.init();
  simpleFOCInitFlag = current_sense_init(); // 初始化电流传感器--ADC的校准和偏差计算
  if (simpleFOCInitFlag != SYS_NORMAL)
  {
    ERR("Current sense init error Not enable motor\n ");
    motor.disable();
    return;
  }
  motor.linkCurrentSense(&current_sense);
  // 检查时候需要自动校准编码器，启用FOC
  if ((direction != -1) && (angle != -1))
  {
    INFO("Success get moto para....\n");
    motor.sensor_direction = (Direction)direction;
    motor.zero_electric_angle = angle;
  }
  else
  {
    INFO("Not get moto para Start autl calibrate");
  }
  // motor.sensor_direction = CW;
  // motor.zero_electric_angle = 5.42;
  current_sense.skip_align = true; // 其中一个电机需要实时校准，因此就直接关闭了电流的校准
  simpleFOCInitFlag = motor.initFOC();
  if (simpleFOCInitFlag == 0)
  {
    ERR("Motor initFOC error Disable Motor driver\n");
    motor.disable();
    simpleFOCInitFlag = SYS_ERROR; // 代表异常，用于循环检测的判断异常
    return;
  }
  simpleFOCInitFlag = SYS_NORMAL;
  // 如果是从新校准的数据则将参数存储在NVS中
  if ((motor.sensor_direction != direction) || (motor.zero_electric_angle != angle))
  {
    int ret = HAL::put_int(MOTO_DIRECTION_PARA, motor.sensor_direction);
    ret += HAL::put_float(MOTO_ANGLE_PARA, motor.zero_electric_angle);
    if (ret != 8)
    {
      ERR("Storage motor para failed(%d)..\n", ret);
    }
    else
    {
      INFO("Store motor para success moto_direction=%d,zero_electric_angle=%f\n",
           motor.sensor_direction, motor.zero_electric_angle);
    }
  }
  motor.target = 0.0f;

  command.add('T', doTarget, "target current");
  command.add('M', doMotor, "my motor");
  _delay(1000);
}
// FOC速度环
void foc_speed_init(int direction = -1, float angle = -1)
{
  INFOLN("foc_speed_init...");
  encode_init(); // 编码器初始化
  driver_init(); // 驱动初始化
  // 连接电机和驱动器
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity;

  // 速度环PID参数设置
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 0.2;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.02;
  motor.voltage_limit = 12;
  // // 角度环PID参数设置
  // motor.P_angle.P = 20;
  // motor.P_angle.I = 0;
  // motor.P_angle.D = 0;
  motor.useMonitoring(Serial);
  // 初始化电机
  motor.init();

  // 检查时候需要自动校准编码器，启用FOC
  if ((direction != -1) && (angle != -1))
  {
    INFO("Success get moto para....\n");
    motor.sensor_direction = (Direction)direction;
    motor.zero_electric_angle = angle;
  }
  else
  {
    INFO("Not get moto para Start autl calibrate");
  }
  // motor.sensor_direction = CW;
  // motor.zero_electric_angle = 5.42;
  simpleFOCInitFlag = motor.initFOC();
  if (simpleFOCInitFlag == 0)
  {
    ERR("Motor initFOC error Disable Motor driver\n");
    motor.disable();
    simpleFOCInitFlag = SYS_ERROR; // 代表异常，用于循环检测的判断异常
    return;
  }
  simpleFOCInitFlag = SYS_NORMAL;
  // 如果是从新校准的数据则将参数存储在NVS中
  if ((motor.sensor_direction != direction) || (motor.zero_electric_angle != angle))
  {
    int ret = HAL::put_int(MOTO_DIRECTION_PARA, motor.sensor_direction);
    ret += HAL::put_float(MOTO_ANGLE_PARA, motor.zero_electric_angle);
    if (ret != 8)
    {
      ERR("Storage motor para failed(%d)..\n", ret);
    }
    else
    {
      INFO("Store motor para success moto_direction=%d,zero_electric_angle=%f\n",
           motor.sensor_direction, motor.zero_electric_angle);
    }
  }
  motor.target = 2.0f;

  command.add('T', doTarget, "target velocity");
  command.add('M', doMotor, "my motor");
  _delay(1000);
}
// FOC位置环
void foc_position_init(int direction = -1, float angle = -1)
{
  INFOLN("FOC position init...");
  encode_init(); // 编码器初始化
  driver_init(); // 驱动初始化
  // 连接电机和驱动器
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::angle;

  // 速度环PID参数设置
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 0.2;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.02;
  motor.voltage_limit = 12;
  // // 角度环PID参数设置
  motor.P_angle.P = 20;
  motor.P_angle.I = 0;
  motor.P_angle.D = 0;
  motor.LPF_angle.Tf = 0;
  motor.useMonitoring(Serial);
  // 初始化电机
  motor.init();

  // 检查时候需要自动校准编码器，启用FOC
  if ((direction != -1) && (angle != -1))
  {
    INFO("Success get moto para....\n");
    motor.sensor_direction = (Direction)direction;
    motor.zero_electric_angle = angle;
  }
  else
  {
    INFO("Not get moto para Start autl calibrate");
  }
  // motor.sensor_direction = CW;
  // motor.zero_electric_angle = 5.42;
  simpleFOCInitFlag = motor.initFOC();
  if (simpleFOCInitFlag == 0)
  {
    ERR("Motor initFOC error Disable Motor driver\n");
    motor.disable();
    simpleFOCInitFlag = SYS_ERROR; // 代表异常，用于循环检测的判断异常
    return;
  }
  simpleFOCInitFlag = SYS_NORMAL;
  // 如果是从新校准的数据则将参数存储在NVS中
  if ((motor.sensor_direction != direction) || (motor.zero_electric_angle != angle))
  {
    int ret = HAL::put_int(MOTO_DIRECTION_PARA, motor.sensor_direction);
    ret += HAL::put_float(MOTO_ANGLE_PARA, motor.zero_electric_angle);
    if (ret != 8)
    {
      ERR("Storage motor para failed(%d)..\n", ret);
    }
    else
    {
      INFO("Store motor para success moto_direction=%d,zero_electric_angle=%f\n",
           motor.sensor_direction, motor.zero_electric_angle);
    }
  }
  motor.target = 1.0f;

  command.add('T', doTarget, "target velocity");
  command.add('M', doMotor, "my motor");
  _delay(1000);
}
#define ZERO_MOTOR_TARGET()                          \
  INFOLN("Previous mode motor reset wait 1 minute"); \
  motor.target = 0;                                  \
  vTaskDelay(pdMS_TO_TICKS(1000));
void HAL::Motor_SwitchMode(UART_RECEIVE_COMMAND mode)
{
  switch (mode)
  {
  case CHOOSE_MOTOR_CURRENT_MODE: // 电流模式
    INFOLN("Switch current mode...");
    ZERO_MOTOR_TARGET();
    motor.torque_controller = TorqueControlType::foc_current; // foc电流力矩控制
    motor.controller = MotionControlType::torque;
    motor.PID_current_q.P = 30; // 5;
    motor.PID_current_q.I = 25; // 300;
    motor.PID_current_q.D = 1;  // 添加了D后，打开后电机,在打开串口条件下，启动会有转动
    motor.PID_current_d.P = 18; // 5;
    motor.PID_current_d.I = 30; // 300;
    motor.PID_current_d.D = 1;  // 打开后电机启动，在打开串口条件下，会有异常转动
    //  低通滤波器
    motor.LPF_current_q.Tf = 0.5;
    motor.LPF_current_d.Tf = 0.5;
    motor.current_limit = 1.0;
    motor.init();
    simpleFOCInitFlag = current_sense_init(); // 初始化电流传感器--ADC的校准和偏差计算
    if (simpleFOCInitFlag != SYS_NORMAL)
    {
      ERR("Current sense init error Not enable motor\n ");
      motor.disable();
      return;
    }
    motor.linkCurrentSense(&current_sense);
    break;
  case CHOOSE_MOTOR_VELOCITY_MODE: // 速度模式
    INFOLN("Switch velocity mode...");
    ZERO_MOTOR_TARGET();
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::velocity;
    // 速度环PID参数设置
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 0.2;
    motor.PID_velocity.D = 0;
    motor.PID_velocity.output_ramp = 1000;
    motor.LPF_velocity.Tf = 0.02;
    motor.voltage_limit = 12;
    motor.init();
    break;
  case CHOOSE_MOTOR_POSITION_MODE: // 位置模式
    INFOLN("Switch position mode...");
    ZERO_MOTOR_TARGET();
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::angle;

    // 速度环PID参数设置
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 0.2;
    motor.PID_velocity.D = 0;
    motor.PID_velocity.output_ramp = 1000;
    motor.LPF_velocity.Tf = 0.02;
    motor.voltage_limit = 12;
    // 角度环PID参数设置
    motor.P_angle.P = 20;
    motor.P_angle.I = 0;
    motor.P_angle.D = 0;
    motor.LPF_angle.Tf = 0;
    motor.init();
    break;
  default:
    ERR("Switch mode error...\n");
    break;
  }
  INFOLN("Switch motot mode complete...");
}
/**************机械角度的转换********************/

// 将0到2π的弧度转换为16位无符号整数
// uint16_t radiansToUint16(float radians)
// {
//   // 确保弧度在0到2π之间
//   while (radians < 0)
//   {
//     radians += TWO_PI;
//     INFOLN("FeedBack angle exceeded MAX angle");
//   }

//   while (radians >= TWO_PI)
//   {
//     radians -= TWO_PI;
//     INFOLN("FeedBack angle less than min angle");
//   }

//   // 映射到0到65535
//   uint16_t value = static_cast<uint16_t>((radians / TWO_PI) * 65535);
//   return value;
// }
// // 将16位无符号整数转换回弧度
// float uint16ToRadians(uint16_t value)
// {
//   // 映射回0到2π
//   float radians = (value / 65535.0f) * (TWO_PI);
//   return radians;
// }
// 将弧度映射到uint16_t，保留两位小数
uint16_t radiansToUint16(float radian)
{
  // 将弧度值转换为一个假定的"百分比"形式
  int32_t scaled = static_cast<int32_t>(radian * 100.0f);
  // 检查是否溢出
  if (scaled < -32768 || scaled > 32767)
  {
    // ERR("Error[radianToUint16]: Value out of range, cannot be accurately represented.\n");
    if (scaled > 32767)
      return 32767;
    if (scaled < -32768)
      return -32768;
  }
  // 映射到uint16_t的范围
  return static_cast<uint16_t>(scaled + 32768);
}

// 将uint16_t映射回弧度，保留两位小数
float uint16ToRadians(uint16_t value)
{
  // 转换回原始的弧度值
  int32_t scaled = static_cast<int32_t>(value) - 32768;
  return scaled / 100.0f;
}
/**************速度的转换********************/
// 将浮点速度转换为uint16_t
uint16_t velocityToUint16(float velocity)
{
  // 假定速度范围为-327.68到+327.67，这允许一定的精度
  // 映射速度到0到65535的范围
  // 首先，调整速度范围到0到65535
  float offsetVelocity = velocity + 327.68;

  // 确保调整后的速度在有效范围内
  if (offsetVelocity < 0)
  {
    offsetVelocity = 0;
    INFOLN("FeedBack velocity less than MAX angle");
  }
  if (offsetVelocity > 655.35)
  {
    INFOLN("FeedBack velocity exceeded MAX angle");
    offsetVelocity = 655.35;
  }

  // 然后，缩放到0到65535
  uint16_t value = static_cast<uint16_t>(offsetVelocity * 100);
  return value;
}

// 将uint16_t转换回浮点速度
float uint16ToVelocity(uint16_t value)
{
  // 将值映射回原始速度范围
  float velocity = (value / 100.0) - 327.68;
  return velocity;
}
/**************电流的转换********************/
uint16_t currentToUint16(float current)
{
  return static_cast<uint16_t>(current * 1000);
}
// 用于打印测试CAN的返回数据是否正常
void test_can_feedback_data(const HAL::MotorFeedData *data, float electrical_angle, float shaft_velocity, float current_sp)
{
  INFOLN("**************MotorData*****************");
  INFO("Angle Send:%f,Receive:%f\n", electrical_angle,
       uint16ToRadians(data->Motor_Data.Rotor_Machinery_Angle));
  INFO("Velocity Send:%f,Receive:%f\n", shaft_velocity,
       uint16ToVelocity(data->Motor_Data.Rotor_Speed));

  INFO("Current Send:%f,Receive:%f\n", current_sp,
       (data->Motor_Data.Toque / 1000.f));
  INFO("Tempetature Send:%d\n", data->Motor_Data.tempetature);
  INFO("SysStatus Send:%d\n", data->Motor_Data.state);
}
// 获取电机的状态
// 用于CAN反馈
void HAL::Motor_GetCurrentState(MotorFeedData &data)
{
  // ESP32是小端模式，高字节存储在高地址，低字节存储在低地址
  // motor.electrical_angle;:electrical_angle//是0~2π的角度范围,而shaft_angle是不断累加的值
  float electrical_angle = motor.shaft_angle;
  float shaft_velocity = motor.shaft_velocity;
  float current_sp = motor.current_sp;

  // 1.转子机械角度
  uint16_t angle = radiansToUint16(electrical_angle); // 放大了1000倍，范围0~2pai
  data.Motor_Data.Rotor_Machinery_Angle = angle;
  // 2.转子速度
  data.Motor_Data.Rotor_Speed = velocityToUint16(shaft_velocity);
  // 3.实际电流
  data.Motor_Data.Toque = currentToUint16(current_sp);
  // 4.MCU温度
  data.Motor_Data.tempetature = MCU_Internal_Temperature_u8();
  // 5.系统状态
  data.Motor_Data.state = getSysState();
  // test_can_feedback_data(&data, electrical_angle, shaft_velocity, current_sp);
}
void HAL::Motor_GetCurrentState(MOTOR_RawData_t &data)
{
  data.target = motor.target;
  data.shaft_angle = motor.shaft_angle;
  data.electrical_angle = motor.electrical_angle;
  data.speed = motor.shaft_velocity;
  data.toque_q = motor.current.q;
  data.toque_d = motor.current.q;
  data.tempetature = MCU_Internal_Temperature_u8();
}
void HAL::Motor_Init()
{
  // 读取参数
  int clock_wise = get_int(MOTO_DIRECTION_PARA, -1);
  float motor_angle = get_float(MOTO_ANGLE_PARA, -1);
  INFO("Get motor param clock_wise(%d) motor_angle(%f)\n", clock_wise, motor_angle);
  foc_current_init(clock_wise, motor_angle);
  // foc_speed_init(clock_wise, motor_angle);
  // foc_position_init(clock_wise, motor_angle);
}
void HAL::Motor_Update(void *parameter)
{
  if (simpleFOCInitFlag == SYS_NORMAL)
  {
    motor.loopFOC();
    // 开环速度运动
    // 使用电机电压限制和电机速度限制
    motor.move();
  }
#if SIMPLEFOC_DEBUG_ENABLE
  // iterative function updating the as5600 internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the as5600 hardware and
  // has to be called before getAngle nad getVelocity
  motor.monitor(); // 可以调用SimpleFOC Studio的上位机观看电机状态
#endif
#if UART_RECEIVE_SELF_METHORD == 0
  command.run();
#endif

  update_receive_can_target();
}
int HAL::Get_MotorState()
{
  return simpleFOCInitFlag;
}
bool HAL::Motor_Disable()
{
  motor.disable();
  return true;
}
void HAL::Motor_ZeroTarget()
{
  motor.target = 0;
}
void HAL::Motor_SetTarget(float data)
{
  motor.target = data;
}