#include "HAL.h"
#include "driver/gpio.h"
static uint8_t System_Status = 0;      // 用于表示当前电机的状态
static uint16_t systemLedSleep = 1000; // 单位为毫秒
static SYS_STATE_SELECT ledMode = SYS_START;
float HAL::MCU_Internal_Temperature_float()
{
    float temperature = temperatureRead();
    return temperature;
}

// 保留两位小数
uint8_t HAL::MCU_Internal_Temperature_u8()
{
    float temperature = temperatureRead();
    return static_cast<uint8_t>(temperature);
}
// 设置当前的系统状态
void HAL::setSysState(SYS_STATE status)
{
    System_Status = static_cast<uint8_t>(status);
}
uint8_t HAL::getSysState()
{
    return System_Status;
}

void HAL::playSysMusic(SYS_STATE_SELECT select)
{
    int choice = static_cast<int>(select);
    switch (choice)
    {
    case SYS_INTERNAL_ERROR:
        cute.play(S_OHOOH);
        break;
    default:
        ERR("Not match case\n");
    }
}
// 调整系统转太灯的闪烁时间
void HAL::selectSysLed(SYS_STATE_SELECT select)
{
    int choice = static_cast<int>(select);
    switch (choice)
    {
    case SYS_INTERNAL_ERROR:
        systemLedSleep = 0; // 系统错误则常亮
        break;
    default:
        ERR("Not match case\n");
    }
}
// 调整模式
static uint8_t led_blink_num = 0;
void HAL::changeLedMode(SYS_STATE_SELECT mode)
{
    switch (mode)
    {
    case CAN_ID_SELECT_MODE:
        led_blink_num = CAN_GetCurrentMotorID();
        break;
    case WIFI_SELECT_MODE:
        break;
    default:
        break;
    }
    ledMode = mode;
}
void HAL::systemStateUpdate()
{
    pinMode(STATUS_LED, OUTPUT);
    // 启用下拉电阻
    gpio_pulldown_en(gpio_num_t(STATUS_LED));
    // 禁用上拉电阻，确保只有下拉电阻被启用
    gpio_pullup_dis(gpio_num_t(STATUS_LED));

    xTaskCreate([](void *)
                {
                    INFOLN("Create systemStateUpdate thread success\n");
                    uint8_t status = 1;
                    unsigned long waitStartTime = 0;
                    bool canWaitStartFlag = false;
                    while (1)
                    {
                        switch(ledMode)
                        {   case WIFI_SELECT_MODE:
                                cute.play(S_WIFI_SELECT); 
                                ledMode = SYS_START;
                                INFOLN("Enter wifi mode");
                            case SYS_START://闪烁提示
                                status = (status==1)?(status=0):(status=1);
                                digitalWrite(STATUS_LED,status);
                                vTaskDelay(pdMS_TO_TICKS(systemLedSleep));
                                break;
                            case CAN_ID_SELECT_MODE://CAN 选择，根据数字闪烁不同次数
                                INFO("Current in can select mode led_blink_num:%d\n",led_blink_num);
                                digitalWrite(STATUS_LED,1);
                                vTaskDelay(pdMS_TO_TICKS(1000));
                                for(int i = 0;i< led_blink_num;i++)
                                {
                                    digitalWrite(STATUS_LED,0);
                                    vTaskDelay(pdMS_TO_TICKS(500));
                                    digitalWrite(STATUS_LED,1);
                                    vTaskDelay(pdMS_TO_TICKS(500));
                                }
                                led_blink_num = 0;
                                vTaskDelay(pdMS_TO_TICKS(1000));
                                ledMode = SYS_START;
                                break;   
                        }
                        
                    } },
                "systemStateUpdate", 4096, NULL, 0, NULL);
}