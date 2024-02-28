# FOC

## 1.将文件夹提交到一个新创建的github服务器

```bash
echo "# miniFOC" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/mailonghua/miniFOC.git
git push -u origin main
```



## 2.CAN协议

一条CAN报文控制4个电机，因此自研电机均用0X300作为ID

接收的报文：

![receive](Readme.assets/20200727133935143.png)

- 一条报文同时控制4个电机
- 四个电机会同时收到报文，然后根据自己的ID进行选取



发送的报文：

标识符：0X300+电机ID，如0X301

![send](Readme.assets/2020072713582544.png)

- 电机温度传输控制芯片内部温度
