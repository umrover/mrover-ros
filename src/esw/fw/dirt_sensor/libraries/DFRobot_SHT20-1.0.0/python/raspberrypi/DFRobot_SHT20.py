# -*- coding: utf-8 -*
'''!
  @file  DFRobot_SHT20.h
  @brief  Define infrastructure of DFRobot_SHT20 class
  @details  Drive SHT20 through this library to get temp and humidity
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license  The MIT License (MIT)
  @author  [Zhangjiawei](jiawei.zhang@dfrobot.com)
  @maintainer  [qsjhyy](yihuan.huang@dfrobot.com)
  @version  V1.0
  @date  2021-12-03
  @url  https://github.com/DFRobot/DFRobot_SHT20
'''
import sys
import time

import smbus

import logging
from ctypes import *


logger = logging.getLogger()
# logger.setLevel(logging.INFO)   # Display all print information
logger.setLevel(logging.FATAL)   # If you don’t want to display too many prints, only print errors, please use this option
ph = logging.StreamHandler()
formatter = logging.Formatter("%(asctime)s - [%(filename)s %(funcName)s]:%(lineno)d - %(levelname)s: %(message)s")
ph.setFormatter(formatter) 
logger.addHandler(ph)


## SHT20 I2C communication error code
ERROR_I2C_TIMEOUT = 998
## SHT20 crc check error code
ERROR_BAD_CRC     = 999
## SHT20 I2C communication address
SHT20_I2C_ADDR    = 0x40

# SHT20 Communication command
TRIGGER_TEMP_MEASURE_HOLD            = 0xE3
TRIGGER_HUMD_MEASURE_HOLD            = 0xE5
TRIGGER_TEMP_MEASURE_NOHOLD          = 0xF3
TRIGGER_HUMD_MEASURE_NOHOLD          = 0xF5
WRITE_USER_REG                       = 0xE6
READ_USER_REG                        = 0xE7
SOFT_RESET                           = 0xFE
USER_REGISTER_RESOLUTION_MASK        = 0x81
USER_REGISTER_RESOLUTION_RH12_TEMP14 = 0x00
USER_REGISTER_RESOLUTION_RH8_TEMP12  = 0x01
USER_REGISTER_RESOLUTION_RH10_TEMP13 = 0x80
USER_REGISTER_RESOLUTION_RH11_TEMP11 = 0x81
USER_REGISTER_END_OF_BATTERY         = 0x40
USER_REGISTER_HEATER_ENABLED         = 0x04
USER_REGISTER_DISABLE_OTP_RELOAD     = 0x02

MAX_WAIT        = 100
DELAY_INTERVAL  = 10
SHIFTED_DIVISOR = 0x988000
MAX_COUNTER     = 10


class DFRobot_SHT20(object):
  '''!
    @brief Define DFRobot_SHT20 basic class
    @details Drive the temp and humidity sensor
  '''

  def __init__(self, i2c_addr=SHT20_I2C_ADDR, bus=1):
    '''!
      @brief Module I2C communication init
      @param i2c_addr I2C communication address
      @param bus I2C bus
    '''
    self._addr = i2c_addr
    self._i2c = smbus.SMBus(bus)

  @property
  def read_humidity(self):
    '''!
      @brief Read the measured data of air humidity
      @return Return the measured air humidity data of float type, unit: %
    '''
    buf = self._read_reg(TRIGGER_HUMD_MEASURE_HOLD, 2)
    humidity = ((buf[0] << 8) | buf[1]) * (125.0 / 65536.0) - 6.0
    return humidity

  @property
  def read_temperature(self):
    '''!
      @brief Read the measured temp data
      @return Return the measured temp data of float type, unit: C
    '''
    buf = self._read_reg(TRIGGER_TEMP_MEASURE_HOLD, 2)
    temperature = ((buf[0] << 8) | buf[1]) * (175.72 / 65536.0) - 46.85
    return temperature

  @property
  def check_SHT20(self):
    '''!
      @brief Check the current status information of SHT20
      @n Status information: End of battery, Heater enabled, Disable OTP reload
      @n Check result: yes, no
    '''
    status = self._read_reg(READ_USER_REG, 1)[0]
    self._show_reslut("End of battery: ", status & USER_REGISTER_END_OF_BATTERY)
    self._show_reslut("Heater enabled: ", status & USER_REGISTER_HEATER_ENABLED)
    self._show_reslut("Disable OTP reload: ", status & USER_REGISTER_DISABLE_OTP_RELOAD)

  def _show_reslut(self, str, val):
    '''!
      @brief Print the check result of the current SHT20 status information
      @param str The status information character string to be printed
      @param val The status information result to be printed
    '''
    print(str, end='')
    if val:
      print("yes")
    else:
      print("no")

  def _read_reg(self, reg, length):
    '''!
      @brief read the data from the register
      @param reg register address
      @param length read data length
      @return read data list
    '''
    return self._i2c.read_i2c_block_data(self._addr, reg, length)
