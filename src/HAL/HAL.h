#ifndef _HAL__H_
#define _HAL__H_
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h> 
#include <SimpleFOC.h>
#include <EEPROM.h>
#include <map>
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "Config.h"
#include "HAL_Def.h"
#include "Log.h"
#include "CuteBuzzerSounds.h" //https://github.com/GypsyRobot/CuteBuzzerSounds



DEBUG_SET_LEVEL(DEBUG_LEVEL_DEBUG);
extern std::string Current_IP_Address;
/*线程等待一定时间,按照一定频率执行线程while(1)*/
#define INIT_TASK_TIME(WAIT_MS) \
        portTickType xLastWakeTime; \
        const portTickType xFrequency = pdMS_TO_TICKS(WAIT_MS); \
        xLastWakeTime = xTaskGetTickCount();   \

#define WAIT_TASK_TIME() \
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
enum FOC_TYPE{
    OPEN_LOOP=0,
};
//EEPROM--单独建立一个文件进行存放
class StorageSYS{
public:
    StorageSYS():start_address(0),current_address(0),end_address(EEPROM_MAX_SIZE){}
    StorageSYS* get_instance(){
        static StorageSYS storage;
        return &storage;
    }
    int write(std::string name, int32_t size,uint8_t*str){
        //判断
        if(size > (end_address - current_address)){
            ERR("Need e2prom too max");
            return -1;
        }
        if((str == NULL)||(size == 0)){
            ERR("Input str is null or size is zero");
        }
        //开始写入
        int ret = find_valid_address(name);
        if(ret == -1){
            //按照空数据写入
            //相关写接口请c参考https://blog.csdn.net/Naisu_kun/article/details/86690493
        }else{
            //重写
        }
    }
    void read(std::string name, int32_t size,uint8_t*str){

    }
private:
    const int32_t start_address;
    const int32_t end_address;
    int32_t current_address;
    std::map<std::string,int> address_map;
    int find_valid_address(std::string name){
        std::map<std::string, int>::iterator it = address_map.find(name);
        if (it != address_map.end()) {
            // 如果键找到了，it->second 就是对应的整数值
            // std::cout << "The value for key '" << key << "' is " << it->second << std::endl;
            INFO("Found valid data:%d\n",it->second);
            return it->second;
        } else {
            // 如果键没找到，说明该键不在map中
            // std::cout << "Key '" << key << "' not found in the map." << std::endl;
            ERR("KEY:%s,not found in the map\n",name);
            return -1;
        }
    }
};
typedef struct{
    uint16_t start_address;//开始地址
    uint16_t end_address;//结束地址
}E2PROM_PARA;
//保存数据的结构体
typedef struct{

}MOTOR_PARA;

namespace HAL
{
    void HAL_Init();
    void HAL_Update();


    //OTA
    void OTA_Init();
    void OTA_Update();

    //Motor-simplefoc
    void Motor_Init();
    void Motor_Update(void *parameter);

    //EEPROM
    void write_motor_parameter();
    void read_motor_parameter();

}
#endif