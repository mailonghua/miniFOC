#include "HAL.h"
// #include <ESP32WiFi.h>
// #include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char *ssid = STASSID;
const char *password = STAPSK;
String Current_IP_Address = "NUll"; // 当前IP地址
void HAL::OTA_Init()
{

    WiFi.mode(WIFI_STA);
    WiFi.setMinSecurity(WIFI_AUTH_WEP);
    // WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    // while (WiFi.waitForConnectResult() != WL_CONNECTED)
    INFOLN("Wait wifi for connect");
    uint16_t retry_times = 60;
    while ((WiFi.status() != WL_CONNECTED) && (retry_times != 0))
    {
        // ERR("Connection Failed! Rebooting...%d", status);
        INFO("Sleep 1s and retry(%s)...\n", ssid);
        retry_times--;
        delay(1000);
        // ESP.restart();
    }
    if ((retry_times == 0) && (WiFi.status() != WL_CONNECTED))
    {
        ERR("Wifi not connect OTA not use\n");
        return;
    }
    ArduinoOTA.onStart([]()
                       {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
        } else {  // U_FS
        type = "filesystem";
        }

        // NOTE: if updating FS this would be the place to unmount FS using FS.end()
        INFO("Start updating %s\r\n",type.c_str()); });
    ArduinoOTA.onEnd([]()
                     { INFO("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { INFO("Progress: %u%%\r", (progress / (total / 100))); });
    ArduinoOTA.onError([](ota_error_t error)
                       {
        ERR("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
        ERR("Auth Failed\r\n");
        } else if (error == OTA_BEGIN_ERROR) {
        ERR("Begin Failed\r\n");
        } else if (error == OTA_CONNECT_ERROR) {
        ERR("Connect Failed\r\n");
        } else if (error == OTA_RECEIVE_ERROR) {
        ERR("Receive Failed\r\n");
        } else if (error == OTA_END_ERROR) {
        ERR("End Failed\r\n");
        } });
    ArduinoOTA.begin();
    INFOLN("Ready...");
    INFOLN("IP address: ");
    INFOLN(WiFi.localIP());
    Current_IP_Address = WiFi.localIP().toString();
}
void HAL::OTA_Update()
{
    ArduinoOTA.handle();
}
static bool otaStatus = false;
static TaskHandle_t xOtaHandle;
void HAL::OTA_SwitchStatus()
{
    otaStatus = !otaStatus;
    if (otaStatus == true) // 准备开启OTA
    {
        INFOLN("Current start ota mode...");

        xTaskCreate(
            [](void *)
            {
                INFOLN("OTA thread create success...");

                HAL::OTA_Init();
                changeLedMode(WIFI_SELECT_MODE); // 播放wifi已经链接的声音
                INIT_TASK_TIME(10);              // 100毫秒-10HZ
                otaStatus = true;
                while (otaStatus)
                {
                    OTA_Update();
                    WAIT_TASK_TIME();
                }
                vTaskDelete(NULL); // 回收本线程
                INFOLN("OTA thread delete success...");
            },
            "OTA_Thread", 4096, NULL, 0, &xOtaHandle);
        return;
    }
    else if (otaStatus == false)
    {
        INFOLN("Current close ota mode...");
        otaStatus = false;
        WiFi.disconnect();               // 断开连接
        WiFi.mode(WIFI_OFF);             // 节省功耗，关闭WIFI模块
        changeLedMode(WIFI_SELECT_MODE); // 播放wifi已经链接的声音
        return;
    }
}