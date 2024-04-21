#include <SPI.h>
#include "mcp2518fd_can.h"
#include <DFRobot_SHT20.h>

// pins for CANBed FD
const int SPI_CS_PIN = 17;
const int CAN_INT_PIN = 7;

mcp2518fd CAN(SPI_CS_PIN); // Set CS pin
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);

union BytesToFloats{
  byte b[8];
  float f[2];
};

void setup() {
    Serial.begin(115200);
    while(!Serial){};

    while (CAN_OK != CAN.begin(CAN_1000K_4M)) {
        Serial.println("CAN init fail, retry...");
        delay(100);
    }
    Serial.println("CAN init ok!");

    sht20.initSHT20();
    delay(100);
}

void loop() {
    BytesToFloats data;
    data.f[0] = sht20.readTemperature();
    data.f[1] = sht20.readHumidity() / 100.0;

    CAN.sendMsgBuf((0x35 << 8) | 0x10, 1, 8, data.b);
    delay(500);
    Serial.println("CAN BUS sendMsgBuf ok!");
    delay(500);
}