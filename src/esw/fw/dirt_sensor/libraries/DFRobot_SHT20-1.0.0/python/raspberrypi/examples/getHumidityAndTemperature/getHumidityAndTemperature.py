# -*- coding: utf-8 -*
'''!
  @file  get_humidity_temperature.py
  @brief  DFRobot's SHT20 Humidity And Temperature Sensor Module
  @details  This example demonstrates how to read the user registers to display resolution and other settings.
  @n  Uses the SHT20 library to display the current humidity and temperature.
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license  The MIT License (MIT)
  @author  [Zhangjiawei](jiawei.zhang@dfrobot.com)
  @maintainer  [qsjhyy](yihuan.huang@dfrobot.com)
  @version  V1.0
  @date  2021-12-03
  @url  https://github.com/DFRobot/DFRobot_SHT20
'''
from __future__ import print_function
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))

from DFRobot_SHT20 import *

'''
  # Module I2C communication init
  # i2c_addr I2C communication address
  # bus I2C bus
'''
sensor = DFRobot_SHT20(i2c_addr = 0x40, bus = 1)


def setup():
  '''
    # Check the current status information of SHT20
    # Status information: End of battery, Heater enabled, Disable OTP reload
    # Check result: yes, no
  '''
  sensor.check_SHT20


def loop():
  '''
    # Read the measured data of air humidity
    # Return the measured air humidity data of float type, unit: %
  '''
  print("Humidity : %.1f %%" %(sensor.read_humidity))

  '''
    # Read the measured temp data
    # Return the measured temp data of float type, unit: C
  '''
  print("Temperature : %.1f C" %(sensor.read_temperature))

  print()
  time.sleep(1)


if __name__ == "__main__":
  setup()
  while True:
    loop()
