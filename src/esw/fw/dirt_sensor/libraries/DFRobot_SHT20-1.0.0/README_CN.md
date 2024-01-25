# DFRobot_SHT20
* [English Version](./README.md)

SHT20 I2C防水型温湿传感器，采用新一代Sensirion湿度和温度传感器，配有4代CMOSens®芯片。
除了配有电容式相对湿度传感器和能隙温度传感器外，该芯片还包含一个放大器、A/D转换器、OTP内存和数字处理单元，可精确测量周边环境温度和空气相对湿度。
相对于上一代SHT1x和SHT7x系列，SHT20具有更好的可靠性和更出色的长期稳定性。

![产品实物图](./resources/images/SHT20.png)


## 产品链接 (https://www.dfrobot.com.cn/goods-1447.html)
    SKU: SEN0227


## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)


## 概述

* 传感器采用双防水技术，内部对电路板进行灌封处理，并且采用特殊灌封材料，对测量值无任何干扰；
* 外壳使用PE防水透气材料，允许水分子进入，阻隔水滴渗入，即使带着电长期浸泡在水中也不会损坏传感器，捞出晾干后，即可正常采集数据。
* 传感器内部带有10k上拉电阻和0.1uf滤波电容，可直接配合Arduino等微控制器使用。
* 推荐配合Gravity: 4Pin传感器转接板一起使用，免去接线烦恼，使用将会变得非常方便。


## 库安装

要使用这个库, 首先下载库文件, 将其粘贴到\Arduino\libraries目录中, 然后打开示例文件夹并在文件夹中运行演示。


## 方法

```C++

  /**
   * @fn initSHT20
   * @brief 初始化函数
   * @return None
   */
  void initSHT20(void);

  /**
   * @fn readHumidity
   * @brief 读取空气湿度测量数据
   * @return 返回float类型的空气湿度测量数据, 单位: %
   */
  float readHumidity(void);

  /**
   * @fn readTemperature
   * @brief 读取温度测量数据
   * @return 返回float类型的温度测量数据, 单位: C
   */
  float readTemperature(void);

  /**
   * @fn checkSHT20
   * @brief 检测SHT20当前状态信息
   * @n 状态信息包括: End of battery, Heater enabled, Disable OTP reload
   * @n 检测结果包括: yes, no
   * @return None
   */
  void checkSHT20(void);

```


## 兼容性

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | :----:
Arduino Uno        |      √       |              |             |
Arduino MEGA2560   |      √       |              |             |
Arduino Leonardo   |      √       |              |             |
FireBeetle-ESP8266 |      √       |              |             |
FireBeetle-ESP32   |      √       |              |             |
FireBeetle-M0      |      √       |              |             |
Micro:bit          |      √       |              |             |


## 历史

- 2017-9-12 - 1.0.0 版本


## 创作者

Written by Zhangjiawei(jiawei.zhang@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))

