#include "HAL.h"
const int rx_queue_size = 10; // Receive Queue size
const int interval = 1000;
static HAL::ReceiveMotorData motor_feed; // 电机的反馈数据
static uint8_t currentMotorID = 0;       // 当前电机的ID-由按键进行设置
QueueHandle_t Motor_Queue;

void HAL::CAN_Init()
{
    // 从NVS取出当前的ID
    CAN_GetCurrentMotorID() = get_int("CAN_ID", 0);
    Motor_Queue = xQueueCreate(1, sizeof(HAL::ReceiveMotorData));
    if (Motor_Queue == NULL)
    {
        while (1)
        {
            ERR("CAN_Init:xQueueCreate return null...\n");
            sleep(1);
        }
    }

    INFOLN("Init CAN Config...");
    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_10, GPIO_NUM_11, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        INFOLN("Twai Driver installed");
    }
    else
    {
        ERR("Failed to install Twai driver\n");
        while (1)
            ;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK)
    {
        INFOLN("Twai Driver started");
    }
    else
    {
        ERR("Failed to start Twai driver\n");
        while (1)
            ;
    }
    // Create thread run Can receive
    xTaskCreate(HAL::CAN_Update, "CAN_Updata", 4096, NULL, 0, NULL);
    // Create thread run Can send motor feedback
#if 1
    xTaskCreate([](void *)
                {
                    INIT_TASK_TIME(5);
                    INFO("Motor Feedback thread create success\n");
                    twai_message_t message;
                    message.identifier = MOTOR_CAN_ID + currentMotorID;
                    message.extd = 0; // 标准帧
                    message.rtr = 0;  // 数据帧
                    message.self = 0; // 不接受自己发送的消息
                    message.data_length_code = 8;
                    MotorFeedData motorFeedData;
                    while (1)
                    {
                        //1.准备数据
                        Motor_GetCurrentState(motorFeedData);
                        for(uint8_t i = 0; i < 8; i++)
                            message.data[i] = motorFeedData.data[i];
                        //2.发送数据
                        int ret = twai_transmit(&message, pdMS_TO_TICKS(1000));
                        // if (ret != ESP_OK)
                        // {
                        //     ERR("Failed(0x%x) to queue message for transmission\n", ret);
                        // }
                        WAIT_TASK_TIME();
                    } },
                "CAN_FeedBack", 4096, NULL, 0, NULL);
#endif
    // 仅用于测试
    // sys_timer *timePtr = sys_timer::get_instance();
    // timePtr->CreateStartTimmer(3000, [](void *)
    //                            { HAL::CAN_Send_Template(); });
}

void HAL::CAN_Receive_Template()
{
    // CAN_frame_t rx_frame;
    // if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
    // {
    //     if (rx_frame.FIR.B.FF == CAN_frame_std)
    //     {
    //         DEBUG("New standard frame");
    //     }

    //     else
    //     {
    //         DEBUG("New extended frame");
    //     }
    //     if (rx_frame.FIR.B.RTR == CAN_RTR)
    //     { // 远程帧
    //         DEBUG(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID, rx_frame.FIR.B.DLC);
    //     }
    //     else
    //     {
    //         DEBUG(" from 0x%08X, DLC %d, Data ", rx_frame.MsgID, rx_frame.FIR.B.DLC);
    //         for (int i = 0; i < rx_frame.FIR.B.DLC; i++)
    //         {
    //             DEBUG("0x%02X ", rx_frame.data.u8[i]);
    //         }
    //         DEBUG("\n");
    //     }
    // }
}
void inline Swap_Byte(uint8_t &Byte1, uint8_t &Byte2)
{
    uint8_t temp;
    temp = Byte1;
    Byte1 = Byte2;
    Byte2 = temp;
}
void HAL::CAN_Receive(ReceiveMotorData *recv_data)
{
    // Wait for message to be received
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK)
    {
        // INFOLN("Message received\n");
    }
    else
    {
        // ERR("Failed to receive message\n");
        return;
    }
    // Process received message
    if (message.extd)
    {
        INFOLN("Message is in Extended Format\n");
    }
    else
    {
        // INFOLN("Message is in Standard Format\n");
        if (MOTOR_CAN_ID == message.identifier) // 检测到接收的数据帧是否是发送给自研电机的
        {
            if (message.data_length_code == 8)
            {
                recv_data->data[0] = message.data[currentMotorID * 2 + 0];
                recv_data->data[1] = message.data[currentMotorID * 2 + 1];
                Swap_Byte(recv_data->data[0], recv_data->data[1]);
                // Overwrite in queue
                xQueueOverwrite(Motor_Queue, recv_data);
            }
            else
            {
                ERR("Recv can motor data not match 8 byte...\n");
            }
        }
    }
    // Debug print
    // INFO("ID is %d\n", message.identifier);
    // if (!(message.rtr)) // 当前不为远程帧
    // {
    //     for (int i = 0; i < message.data_length_code; i++)
    //     {
    //         INFO("Data byte %d = %d\n", i, message.data[i]);
    //     }
    // }
}

void HAL::CAN_Update(void *)
{
    INIT_TASK_TIME(5); // 5MS
    INFO("Start CAN_Update thread\n");
    while (1)
    {
        CAN_Receive(&motor_feed);
        WAIT_TASK_TIME();
    }
}
/*获得当前已经存储电机反馈数据*/
// Old api not use 新机制已经使用队列传递
HAL::ReceiveMotorData *HAL::CAN_Get_Data()
{
    return &motor_feed;
}
/*
    电机CAN ID 0X200和0x1FF各自对应控制4个ID的电调
*/
void HAL::CAN_Send(SendMotorData &send_data, uint32_t MsgID)
{
    twai_message_t message;
    message.identifier = MOTOR_CAN_ID + currentMotorID;
    message.extd = 0; // 标准帧
    message.rtr = 0;  // 数据帧
    message.self = 0; // 不接受自己发送的消息
    message.data_length_code = 8;
    message.data[0] = send_data.Feed_Data.data[1];
    message.data[1] = send_data.Feed_Data.data[0];
    message.data[2] = send_data.Feed_Data.data[3];
    message.data[3] = send_data.Feed_Data.data[2];
    message.data[4] = send_data.Feed_Data.data[5];
    message.data[5] = send_data.Feed_Data.data[4];
    message.data[6] = send_data.Feed_Data.data[7];
    message.data[7] = send_data.Feed_Data.data[6];
    // Queue message for transmission
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
    {
        INFO("Message queued for transmission\n");
    }
    else
    {
        ERR("Failed to queue message for transmission\n");
    }
}

void HAL::CAN_Send_Template()
{
    twai_message_t message;
    message.identifier = MOTOR_CAN_ID + currentMotorID;
    message.extd = 0; // 标准帧
    message.rtr = 0;  // 数据帧
    message.self = 0; // 不接受自己发送的消息
    message.data_length_code = 8;

    message.data[0] = 0x00;
    message.data[1] = 0x01;
    message.data[2] = 0x02;
    message.data[3] = 0x03;
    message.data[4] = 0x04;
    message.data[5] = 0x05;
    message.data[6] = 0x06;
    message.data[7] = 0x07;

    // Queue message for transmission
    int ret = twai_transmit(&message, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK)
    {
        // INFO("Message queued for transmission\n");
    }
    else
    {
        ERR("Failed(0x%x) to queue message for transmission\n", ret);
    }
}
// 用于通过按键或其他外设来设置当前的电机ID，从0开始
uint8_t &HAL::CAN_GetCurrentMotorID()
{
    return currentMotorID;
}
void HAL::CAN_SetCurrentMotorID(uint8_t id)
{
    currentMotorID = id;
}