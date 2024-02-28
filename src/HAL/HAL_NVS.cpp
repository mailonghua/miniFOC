#include "HAL.h"
static Preferences preferences;
static bool init_success_flag = false;
void test()
{
    // 假设我们要存储一个浮点数
    float temperature = 25.75;
    bool writeSuccessful = HAL::put_float("temperature", temperature);
    if (writeSuccessful)
    {
        Serial.println("Temperature stored successfully.");
    }
    else
    {
        Serial.println("Failed to store temperature.");
    }

    // 读取浮点数
    float storedTemperature = HAL::get_float("temperature", -1.0); // 默认值为-1.0
    Serial.print("Stored temperature is: ");
    Serial.println(storedTemperature);
}
int HAL::NVS_Init()
{
    INFO("Start init nvs...\n");
    if (!preferences.begin("miniFOC", false))
    {
        ERR("Failed to initialize NVS\n");
        return ERROR;
    }
    else
    {
#if 0
        if (NVC_Clear())
        {
            INFO("Success clear all data in nvs\n");
        }
        else
        {
            ERR("Clear all nvs data success\n");
        }
#endif

        init_success_flag = true;
        // test();
        return SUCCESS;
    }
}
void HAL::NVS_End()
{
    preferences.end();
    init_success_flag = false;
    INFO("Stop init nvs...\n");
}
float HAL::put_float(const char *key, float_t value)
{
    if (init_success_flag == false)
    {
        ERR("Please init Preferences...\n");
        return 0.0;
    }
    return preferences.putFloat(key, value);
}
float HAL::get_float(const char *key, float_t defaultValue)
{
    if (init_success_flag == false)
    {
        ERR("Please init Preferences...\n");
        return 0.0;
    }
    return preferences.getFloat(key, defaultValue);
}
int HAL::put_int(const char *key, int32_t value)
{
    if (init_success_flag == false)
    {
        ERR("Please init Preferences...\n");
        return 0.0;
    }
    return preferences.putInt(key, value);
}
int HAL::get_int(const char *key, int32_t defaultValue)
{
    if (init_success_flag == false)
    {
        ERR("Please init Preferences...\n");
        return 0.0;
    }
    return preferences.getInt(key, defaultValue);
}
bool HAL::NVC_Clear()
{
    return preferences.clear();
}