#include "HAL.h"
/*按键输入*/
void HAL::Input_Key_Init()
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
                        changeLedMode(CAN_ID_SELECT_MODE);
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
/*串口输入的初始化*/
void HAL::Uart_Receive_Init()
{
}