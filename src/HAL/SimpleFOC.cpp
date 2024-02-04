
#include "HAL.h"
#include <SimpleFOC.h>

static MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);
static BLDCMotor motor = BLDCMotor(7);
static BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 7, 4);
static InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.f, 13, 14);
static Commander command = Commander(Serial);

void doTarget(char *cmd) { command.scalar(&motor.target, cmd); }
void doMotor(char *cmd) { command.motor(&motor, cmd); }


void encode_init(){//编码器初始化
  // 1.磁编码器设置
  Wire.begin(1, 0, (uint32_t)400000);
  Wire.setClock(400000); // 400000
  // initialise magnetic as5600 hardware
  as5600.init(&Wire);
  motor.linkSensor(&as5600);
}
//驱动器初始化
void driver_init(){
  driver.voltage_power_supply = 24;
  driver.init();
}
//电流传感器初始化
void current_sense_init(){
  if (current_sense.init())
    Serial.println("Current sense init success!");
  else
  {
    Serial.println("Current sense init failed!");
    return;
  }
  //结合硬件，可以看到当前的版本的B相的检测电阻和检测引脚方向是反的，因此这里取反
  current_sense.gain_b *= -1;
}

//FOC电流环
void foc_current_init()
{
    encode_init();//编码器初始化
    driver_init();//驱动初始化
    current_sense.linkDriver(&driver);
    // 连接电机和驱动器
    motor.linkDriver(&driver);
    motor.torque_controller = TorqueControlType::foc_current; // foc电流力矩控制
    motor.controller = MotionControlType::torque;
    // 电流环的q和d的PID
    motor.PID_current_q.P = 20; // 5;
    motor.PID_current_q.I = 20; // 300;
    motor.PID_current_d.P = 18; // 5;
    motor.PID_current_d.I = 20; // 300;
    // 低通滤波器
    motor.LPF_current_q.Tf = 0.5;
    motor.LPF_current_d.Tf = 0.5;
    motor.current_limit = 1.0;
    motor.useMonitoring(Serial);
    // 初始化电机
    motor.init();
    current_sense_init();//初始化电流传感器--ADC的校准和偏差计算
    motor.linkCurrentSense(&current_sense);
    // 校准编码器，启用FOC
    motor.initFOC();
    motor.target = 0.0f;

    command.add('T', doTarget, "target current");
    command.add('M', doMotor, "my motor");
    _delay(1000);
}
//FOC速度环
void foc_speed_init()
{

}
//FOC位置环
void foc_position_init()
{

}
void HAL::Motor_Init()
{
    foc_current_init();
}
void HAL::Motor_Update(void *parameter)
{
  motor.loopFOC();
  // 开环速度运动
  // 使用电机电压限制和电机速度限制
  motor.move();
  // iterative function updating the as5600 internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the as5600 hardware and
  // has to be called before getAngle nad getVelocity
  motor.monitor();
  command.run();  
}