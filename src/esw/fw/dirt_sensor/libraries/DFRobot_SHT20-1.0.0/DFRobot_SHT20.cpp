/*!
 * @file  DFRobot_SHT20.cpp
 * @brief  Define the basic structure of class DFRobot_SHT20
 * @details  Drive SHT20 through this library to get temp and humidity
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [Zhangjiawei](jiawei.zhang@dfrobot.com)
 * @maintainer  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2021-12-03
 * @url  https://github.com/DFRobot/DFRobot_SHT20
 */
#include "DFRobot_SHT20.h"

DFRobot_SHT20::DFRobot_SHT20(TwoWire *pWire, uint8_t sht20Addr)
{
  _addr = sht20Addr;
  _pWire = pWire;
}

void DFRobot_SHT20::initSHT20()
{
    _pWire->begin();
}

float DFRobot_SHT20::readHumidity(void)
{
    uint16_t rawHumidity = readValue(TRIGGER_HUMD_MEASURE_NOHOLD);
    if(rawHumidity == ERROR_I2C_TIMEOUT || rawHumidity == ERROR_BAD_CRC){
        return(rawHumidity);
    }
    float tempRH = rawHumidity * (125.0 / 65536.0);
    float rh = tempRH - 6.0;
    return (rh);
}

float DFRobot_SHT20::readTemperature(void)
{
    uint16_t rawTemperature = readValue(TRIGGER_TEMP_MEASURE_NOHOLD);
    if(rawTemperature == ERROR_I2C_TIMEOUT || rawTemperature == ERROR_BAD_CRC){
        return(rawTemperature);
    }
    float tempTemperature = rawTemperature * (175.72 / 65536.0);
    float realTemperature = tempTemperature - 46.85;
    return (realTemperature);
}

void DFRobot_SHT20::checkSHT20(void)
{
    byte reg = readUserRegister();
    showReslut("End of battery: ", reg & USER_REGISTER_END_OF_BATTERY);
    showReslut("Heater enabled: ", reg & USER_REGISTER_HEATER_ENABLED);
    showReslut("Disable OTP reload: ", reg & USER_REGISTER_DISABLE_OTP_RELOAD);
}

void DFRobot_SHT20::setResolution(byte resolution)
{
    byte userRegister = readUserRegister();
    userRegister &= B01111110;
    resolution &= B10000001;
    userRegister |= resolution;
    writeUserRegister(userRegister);
}

byte DFRobot_SHT20::readUserRegister(void)
{
    byte userRegister;
    _pWire->beginTransmission(_addr);
    _pWire->write(READ_USER_REG);
    _pWire->endTransmission();
    _pWire->requestFrom(_addr, (uint8_t)1);
    userRegister = _pWire->read();
    return (userRegister);
}

void DFRobot_SHT20::writeUserRegister(byte val)
{
    _pWire->beginTransmission(_addr);
    _pWire->write(WRITE_USER_REG);
    _pWire->write(val);
    _pWire->endTransmission();
}

void DFRobot_SHT20::showReslut(const char *prefix, int val)
{
    Serial.print(prefix);
    if(val){
        Serial.println("yes");
    }else{
        Serial.println("no");
    }
}

byte DFRobot_SHT20::checkCRC(uint16_t message_from_sensor, uint8_t check_value_from_sensor)
{
    uint32_t remainder = (uint32_t)message_from_sensor << 8;
    remainder |= check_value_from_sensor;
    uint32_t divsor = (uint32_t)SHIFTED_DIVISOR;
    for(int i = 0 ; i < 16 ; i++){
        if(remainder & (uint32_t)1 << (23 - i)){
            remainder ^= divsor;
        }
        divsor >>= 1;
    }
    return (byte)remainder;
}

uint16_t DFRobot_SHT20::readValue(byte cmd)
{
    _pWire->beginTransmission(_addr);
    _pWire->write(cmd);
    if(0 != _pWire->endTransmission()){   // Used Wire.endTransmission() to end a slave transmission started by beginTransmission() and arranged by write().
      return (ERROR_I2C_TIMEOUT);
    }
    byte toRead;
    byte counter;
    for(counter = 0, toRead = 0 ; counter < MAX_COUNTER && toRead != 3; counter++){
        delay(DELAY_INTERVAL);
        toRead = _pWire->requestFrom(_addr, (uint8_t)3);
    }
    if(counter == MAX_COUNTER){
        return (ERROR_I2C_TIMEOUT);
    }
    byte msb, lsb, checksum;
    msb = _pWire->read();
    lsb = _pWire->read();
    checksum = _pWire->read();
    uint16_t rawValue = ((uint16_t) msb << 8) | (uint16_t) lsb;
    if(checkCRC(rawValue, checksum) != 0){
        return (ERROR_BAD_CRC);
    }
    return rawValue & 0xFFFC;
}
