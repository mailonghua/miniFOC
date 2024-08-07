# FOC

## 1.说明

本项目基于实现FOC电流环控制，同时包含CAN反馈、UART控制，同时支持OTA，曲线绘制、NVS参数保存等，同时支持速度环和位置环的demo演示

## 2.CAN协议

一条CAN报文控制4个电机，因此自研电机均用0X300作为ID

**接收的报文：**

![receive](Readme.assets/20200727133935143.png)

- 一条报文同时控制4个电机

- 四个电机会同时收到报文，然后根据自己的ID进行选取

- 电流值是放大了1000倍

  如接收到了300，则代表300/1000 = 0.3A，如在CAN控制器中发送的发送

<img src="Readme.assets/image-20240421133928423.png" alt="image-20240421133928423" style="zoom:80%;" />

- 请注意，当调整电机ID时，下发的数据所在位置也要对应选择，
  - 比如ID为0，对应就是Byte0~Byte1
  - 比如ID为1，对应就是Byte2~Byte3
  - ....

**反馈的报文：**

标识符：0X300+电机ID，如0X301

![send](Readme.assets/2020072713582544.png)

- 电机温度代表传控制芯片内部温度



## 3.按键的控制

### 3.1.调整CAN ID

短按用来调整CAN ID，LED灯的闪烁的次数，用来调整ID从0~4，数字就是LED闪烁的次数

​	那么对应的CAN ID就是 CAN_ID = 0x300 + id构成

注意：当调节完ID后，会记录到SOC内部的NVS中，需要重新启动才会生效

### 2.开启或关闭WIFI-用于OTA

2秒长按会开启或关闭wifi模式，当链接到网络则会有滴滴提示音

### 3.清除所有的NVS

大于3.5秒长按会清除所有NVS中保存的数据

此时重新上电，电机则会进行校准状态，并将校准数据写入到NVS中，同时CAN ID为默认的0x300

## 4.串口指令

| 指令            | 说明                          | 返回值                           |
| --------------- | ----------------------------- | -------------------------------- |
| 0x5500          | 获取系统状态                  | 0：Success<br />1:Fail           |
| 0x5501          | 开启WIFI                      | 0：Success<br />1:Fail           |
| 0x5502          | *关闭WIFI*                    | 0：Success<br />1:Fail           |
| 0x5503          | 清空NVS                       | 0：Success<br />1:Fail           |
| 0x5504          | 开启BLE\<todo>                | 0：Success<br />1:Fail           |
| 0x5505          | 关闭BLE\<todo>                | 0：Success<br />1:Fail           |
| 0x5506          | 设置重新修改WIFI和SSID\<todo> | 0：Success<br />1:Fail           |
| 0x5507          | 失能电机                      | 0：Success<br />1:Fai            |
| TTxx.xx[字符串] | 发送目标值                    | NULL                             |
| 0x5511          | 开启/关闭波形输出             | NULL                             |
| 0x5508          | *力矩模式*                    | 补充：其他模式无法切换到力矩模式 |
| 0x5509          | *速度模式*                    | 其他模式可切换到该模式           |
| 0x5510          | *位置模式*                    | 其他模式可切换到该模式           |
| ID0[字符串]     | 设置电机ID                    | 比如当前设置电机id为0x300+0      |
| 0x5513          | 电机系统重启                  |                                  |



## 注意事项

1.OTA升级后再使用USB刷新固件

如果在使用了OTA进行升级后，再使用USB进行固件刷新，请先试用platormIO的清除整个Flash再刷新才会生效，OTA在更新时会修改使能缓冲区

2.清空eps32S3 Flash

 python -m esptool --chip esp32-S3  erase_flash

3.提交

```
echo "# miniFOC" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/mailonghua/miniFOC.git
git push -u origin main
```

 
