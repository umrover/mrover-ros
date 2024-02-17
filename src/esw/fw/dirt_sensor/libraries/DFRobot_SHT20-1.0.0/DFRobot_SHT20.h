/*!
 * @file  DFRobot_SHT20.h
 * @brief  Define infrastructure of DFRobot_SHT20 class
 * @details  Drive SHT20 through this library to get temp and humidity
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [Zhangjiawei](jiawei.zhang@dfrobot.com)
 * @maintainer  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2021-12-03
 * @url  https://github.com/DFRobot/DFRobot_SHT20
 */
#ifndef __DFRobot_SHT20_H__
#define __DFRobot_SHT20_H__

#include <Arduino.h>
#include <Wire.h>

// #define ENABLE_DBG   //!< Open the macro and you can see the details of the program
#ifdef ENABLE_DBG
  #define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
  #define DBG(...)
#endif

#define ERROR_I2C_TIMEOUT                     998
#define ERROR_BAD_CRC                         999
#define SHT20_I2C_ADDR                        0x40

#define TRIGGER_TEMP_MEASURE_HOLD             0xE3
#define TRIGGER_HUMD_MEASURE_HOLD             0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD           0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD           0xF5
#define WRITE_USER_REG                        0xE6
#define READ_USER_REG                         0xE7
#define SOFT_RESET                            0xFE
#define USER_REGISTER_RESOLUTION_MASK         0x81
#define USER_REGISTER_RESOLUTION_RH12_TEMP14  0x00
#define USER_REGISTER_RESOLUTION_RH8_TEMP12   0x01
#define USER_REGISTER_RESOLUTION_RH10_TEMP13  0x80
#define USER_REGISTER_RESOLUTION_RH11_TEMP11  0x81
#define USER_REGISTER_END_OF_BATTERY          0x40
#define USER_REGISTER_HEATER_ENABLED          0x04
#define USER_REGISTER_DISABLE_OTP_RELOAD      0x02

#define MAX_WAIT                              100
#define DELAY_INTERVAL                        10
#define SHIFTED_DIVISOR                       0x988000
#define MAX_COUNTER                           (MAX_WAIT/DELAY_INTERVAL)

class DFRobot_SHT20
{
public:
  /**
   * @fn DFRobot_SHT20
   * @brief Constructor
   * @param pWire Wire object is defined in Wire.h, so just use &Wire, and the methods in Wire can be pointed to and used
   * @param sht20Addr I2C communication address of SHT20
   * @return None
   */
  DFRobot_SHT20(TwoWire *pWire=&Wire, uint8_t sht20Addr=SHT20_I2C_ADDR);

  /**
   * @fn initSHT20
   * @brief Init function
   * @return None
   */
  void initSHT20(void);

  /**
   * @fn readHumidity
   * @brief Read the measured data of air humidity
   * @return Return the measured air humidity data of float type, unit: %
   */
  float readHumidity(void);

  /**
   * @fn readTemperature
   * @brief Read the measured temp data
   * @return Return the measured temp data of float type, unit: C
   */
  float readTemperature(void);

  /**
   * @fn checkSHT20
   * @brief Check the current status information of SHT20
   * @n Status information: End of battery, Heater enabled, Disable OTP reload
   * @n Check result: yes, no
   * @return None
   */
  void checkSHT20(void);

protected:
  /**
   * @fn setResolution
   * @brief Set measurement resolution of sht20
   * @param resBits The measurement resolution mode of SHT20
   * @return None
   */
  void setResolution(byte resBits);

  /**
   * @fn readUserRegister
   * @brief Read user register
   * @return The read byte
   */
  byte readUserRegister(void);

  /**
   * @fn writeUserRegister
   * @brief Write user register
   * @param val The written byte
   * @return None
   */
  void writeUserRegister(byte val);

  /**
   * @fn showReslut
   * @brief Print the check result of the current SHT20 status information
   * @param prefix The status information character string to be printed
   * @param val The status information result to be printed
   * @return None
   */
  void showReslut(const char *prefix, int val);

  /**
   * @fn checkCRC
   * @brief Calculate and check crc
   * @param message_from_sensor The data read from the sensor
   * @param check_value_from_sensor The check value read from the sensor
   * @return Return the check result, 0 indicate check passed
   */
  byte checkCRC(uint16_t message_from_sensor, uint8_t check_value_from_sensor);

  /**
   * @fn readValue
   * @brief Read the register value through I2C bus
   * @param cmd The read command to be sent
   * @return Return the read 16-bit data
   */
  uint16_t readValue(byte cmd);

private:
  uint8_t _addr;   // I2C communication address of SHT20
  TwoWire *_pWire;   // The pointer to I2C communication method
};

#endif
