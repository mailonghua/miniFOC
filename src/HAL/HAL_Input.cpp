#include "HAL.h"
static enum UART_RECEIVE_COMMAND currentMotorStatus = CHOOSE_MOTOR_CURRENT_MODE;
UART_RECEIVE_COMMAND HAL::Get_CurrentMotorStatus()
{
    return currentMotorStatus;
}
/*按键输入*/
void HAL::StartInputKetDetectTask()
{
    pinMode(KEY, INPUT_PULLUP);
    xTaskCreate(
        [](void *)
        {
            unsigned long pressed_time = 0;
            unsigned long released_time = 0;
            uint8_t prev_button_state = LOW;
            uint8_t button_state = LOW;
            bool button_is_press = false;
            INFOLN("Button  thread create success...");
            INIT_TASK_TIME(KEY_DETECT_THREAD_DELAY); // 50ms检测一次
            while (1)
            {
                button_state = digitalRead(KEY);
                // 按键按下
                if ((button_state == LOW) && (prev_button_state == HIGH))
                {
                    button_is_press = true;
                    pressed_time = millis();
                }
                // 按键松开
                if ((button_is_press == true) && (button_state == HIGH) && (prev_button_state == LOW))
                {
                    button_is_press = false;
                    released_time = millis();
                    unsigned long diffTime = released_time - pressed_time;
                    // 判断对应属于的事件
                    // 短按-CAN
                    if (diffTime < (SELECT_CAN_ID_DELAY_MAX * KEY_DETECT_THREAD_DELAY))
                    {
                        INFOLN("CAN select button press...");
                        NVS_Init(); // 打开NVS
                        if (CAN_GetCurrentMotorID() < 4)
                            CAN_GetCurrentMotorID()++; // CAN_GetCurrentMotorID返回的是一个引用
                        else
                            CAN_GetCurrentMotorID() = 0;
                        put_int("CAN_ID", static_cast<int>(CAN_GetCurrentMotorID()));
                        NVS_End(); // 关闭NVS
                        changeShowSysMode(CAN_ID_SELECT_MODE);
                    }
                    // 长按-WIFI
                    else if ((diffTime > (SELECT_OPEN_WIFI_MIN * KEY_DETECT_THREAD_DELAY)) && (diffTime < (SELECT_CLEAR_NVS_MIN * KEY_DETECT_THREAD_DELAY)))
                    {
                        INFOLN("WIFI select button press...");
                        OTA_SwitchStatus();
                    }
                    else if (diffTime > (SELECT_CLEAR_NVS_MIN * KEY_DETECT_THREAD_DELAY))
                    {
                        INFOLN("Clear NVS data...");
                        NVS_Init();
                        NVC_Clear();
                        NVS_End();
                    }
                }
                // save the the last state
                prev_button_state = button_state;
                WAIT_TASK_TIME();
            }
        },
        "KeyDetectThread", 2048, NULL, 0, NULL);
}

// 串口处理的任务
float valueTaget = 0;
uint8_t feedBackStr[3] = {0x55, 0x00};
static UART_RECEIVE_COMMAND currentCMD; // 当前指令

// 用于波形debug
static TaskHandle_t waveDebugHandle;
static bool waveDebugFlag = false;
static void Uart_Receive_Case_Switch()
{
    uint16_t size = Serial.available();
    // 1.判断数据大小
    if (size < 2)
    {
        ERR("Receive uart data error size:%d,data:0x%x\n", size, Serial.read());
        return;
    }
    uint8_t *bufferPtr = static_cast<uint8_t *>(malloc(size));
    // 2.获取指令的数据
    if (bufferPtr != NULL)
    {
        Serial.read(bufferPtr, size);
    }
    else
    {
        ERR("Uart receive buffer error, programe return...\n");
        return;
    }
    // 3.处理数据
    // 串口发送的数据高字节先发送，低字节后发送
    uint16_t cmd = bufferPtr[0];
    cmd = cmd << 8 | bufferPtr[1];

    switch (cmd)
    {
    case GET_SYS_STATE_CMD:
        INFOLN("Get system state...");
        feedBackStr[2] = HAL::getSysState();
        Serial.write(feedBackStr, 3);
        break;
    case CLOSE_WIFI_PWR_CMD:
    case OPEN_WIFI_PWR_CMD:
        INFOLN("Switch wifi state...");
        feedBackStr[1] = 0x01;
        feedBackStr[2] = 0x0;
        HAL::OTA_SwitchStatus();
        Serial.write(feedBackStr, 3);
        break;

    case CLEAR_NVS_CMD:
        INFOLN("Clear NVS parameter...");
        feedBackStr[1] = 0x03;
        feedBackStr[2] = 0x0;
        HAL::NVS_Init();
        HAL::NVC_Clear();
        HAL::NVS_End();
        Serial.write(feedBackStr, 3);
        break;
    case OPEN_BLE_PWR_CMD:
        /* code */
        break;
    case CLOSE_BLE_PWR_CMD:
        /* code */
        break;
    case SET_WIFI_SSID_PASSWD:
        /* code */
        break;
    case DISABLE_MOTOR_DISABLE:
        INFOLN("Disable Motor...");
        HAL::Motor_Disable();
        break;
    case CHOOSE_MOTOR_CURRENT_MODE:
        INFOLN("Set motor for current mode");
        currentMotorStatus = CHOOSE_MOTOR_CURRENT_MODE;
        HAL::Motor_SwitchMode(CHOOSE_MOTOR_CURRENT_MODE);
        break;
    case CHOOSE_MOTOR_VELOCITY_MODE:
        INFOLN("Set motor for velocity mode");
        currentMotorStatus = CHOOSE_MOTOR_VELOCITY_MODE;
        HAL::Motor_SwitchMode(CHOOSE_MOTOR_VELOCITY_MODE);
        break;
    case CHOOSE_MOTOR_POSITION_MODE:
        INFOLN("Set motor for position mode");
        currentMotorStatus = CHOOSE_MOTOR_POSITION_MODE;
        HAL::Motor_SwitchMode(CHOOSE_MOTOR_POSITION_MODE);
        break;
    case SET_MOTOR_TARGET:
    {
        INFO("Set motor target(%d)...\n", size);
        if (size <= 2)
        {
            ERR("No valid target data...\n");
            for (int i = 0; i < size; i++)
            {
                INFO("0x%x ", bufferPtr[i]);
            }
            break;
        }
        char *dataPtr = static_cast<char *>(malloc(size - 1));
        for (int i = 2; i < size; i++)
        {
            dataPtr[i - 2] = static_cast<char>(bufferPtr[i]);
        }
        dataPtr[size - 2] = '\0'; // 使用atof需要手动添加结束符
        INFO("Receive motor target:%s\n", dataPtr);
        HAL::Motor_SetTarget(atof(dataPtr));
        free(dataPtr);
        break;
    }
    case OPEN_MOTOR_WAVAE_DEBUG:
    {
        if (waveDebugFlag == false)
        {
            xTaskCreate(
                [](void *)
                {
                    waveDebugFlag = true;
                    INFOLN("Motor wave debug thread create success...");
                    INIT_TASK_TIME(50);
                    while (waveDebugFlag)
                    {
                        HAL::MOTOR_RawData_t data;
                        HAL::Motor_GetCurrentState(data);
                        Serial.printf("%f,%f,%f,%f,%f,%f,%f\n",
                                      data.target,
                                      data.shaft_angle,
                                      data.electrical_angle,
                                      data.speed,
                                      data.toque_q,
                                      data.toque_d,
                                      data.tempetature);
                        WAIT_TASK_TIME();
                    }
                    INFOLN("Close motor wave debug thread complete");
                    vTaskDelete(NULL);
                },
                "motor_debug_thread", 4096, NULL, 0, &waveDebugHandle);
        }
        else
        {
            INFOLN("Start Close motor wave debug thread");
            waveDebugFlag = false;
        }
    }
    case SET_MOTOR_CAN_ID:
    {
        INFO("Set motor CAN ID\n");
        if (size <= 2)
        {
            ERR("No valid target data...\n");
            for (int i = 0; i < size; i++)
            {
                INFO("0x%x ", bufferPtr[i]);
            }
            break;
        }
        char *dataPtr = static_cast<char *>(malloc(size - 1));
        for (int i = 2; i < size; i++)
        {
            dataPtr[i - 2] = static_cast<char>(bufferPtr[i]);
        }
        dataPtr[size - 2] = '\0'; // 使用atof需要手动添加结束符
        INFO("Receive motor ID:%s\n", dataPtr);
        // 设置CAN ID并且更新NVS，并进行重启
        HAL::NVS_Init(); // 打开NVS
        int32_t canID = atoi(dataPtr);
        HAL::CAN_GetCurrentMotorID() = canID;
        HAL::put_int("CAN_ID", canID);
        HAL::NVS_End(); // 关闭NVS
        free(dataPtr);
        INFO("Motor system reboot Set CAN ID=0x%x\n", canID);
        REBOOT_SYS();
    }
    case SET_SYSTEM_REBOOT:
    {
        INFO("Motor system reboot\n");
        HAL::Motor_Disable();
        sleep(1);
        REBOOT_SYS();
    }

    break;
    default:
        ERR("Not match cmd(0x%x) receive data:\n", cmd);
        for (int i = 0; i < size; i++)
        {
            Serial.printf("0x%x ", bufferPtr[i]);
        }
        Serial.printf("\n------\n");
        break;
    }
    // 释放内存
    free(bufferPtr);
}
/*串口输入的初始化*/
void HAL::Uart_Receive_IRQ_Register()
{
    Serial.onReceive(Uart_Receive_Case_Switch);
    INFOLN("Register uart receive callback complete...");
}