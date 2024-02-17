/*!
 * @file  getHumidityAndTemperature.ino
 * @brief  DFRobot's SHT20 Humidity And Temperature Sensor Module
 * @details  This example demonstrates how to read the user registers to display resolution and other settings.
 * @n  Uses the SHT20 library to display the current humidity and temperature.
 * @n  Open serial monitor at 9600 baud to see readings.
 * @n  Errors 998 if not sensor is detected. Error 999 if CRC is bad.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [Zhangjiawei](jiawei.zhang@dfrobot.com)
 * @maintainer  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2021-12-03
 * @url  https://github.com/DFRobot/DFRobot_SHT20
 */
#include "DFRobot_SHT20.h"

/**
 * Hardware Connections:
 * -VCC = 3.3V
 * -GND = GND
 * -SDA = A4 (use inline 330 ohm resistor if your board is 5V)
 * -SCL = A5 (use inline 330 ohm resistor if your board is 5V)
 */
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);

void setup()
{
  Serial.begin(115200);

  // Init SHT20 Sensor
  sht20.initSHT20();
  delay(100);
  Serial.println("Sensor init finish!");

  /**
   * Check the current status information of SHT20
   * Status information: End of battery, Heater enabled, Disable OTP reload
   * Check result: yes, no
   */
  sht20.checkSHT20();
}

void loop()
{
  /**
   * Read the measured data of air humidity
   * Return the measured air humidity data of float type, unit: %
   */
  float humd = sht20.readHumidity();

  /**
   * Read the measured temp data
   * Return the measured temp data of float type, unit: C
   */
  float temp = sht20.readTemperature();

  Serial.print("Time:");
  Serial.print(millis());   // Get the system time from Arduino
  Serial.print(" Temperature:");
  Serial.print(temp, 1);   // Only print one decimal place
  Serial.print("C");
  Serial.print(" Humidity:");
  Serial.print(humd, 1);   // Only print one decimal place
  Serial.print("%");
  Serial.println();

  delay(1000);
}
