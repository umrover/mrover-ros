#include <ACAN2517FD.h>
#include <DFRobot_SHT20.h>


DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);

const int SPI_CS_PIN = 17;
const int CAN_INT_PIN = 7;

// TODO: change CAN object to use new library
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin

// Store data into struct from I2C, send this out to Jetson
struct DirtData {
  float temperature;
  float humidity;
}

void setup() {
  sht20.initSHT20();
  delay(100);


  Serial.begin(115200);
  while(!Serial){};

  while (CAN_OK != CAN.begin(CAN_1000K_4M)) {
      Serial.println("CAN init fail, retry...");
      delay(100);
  }
  Serial.println("CAN init ok!");
}

void loop() {
  // TODO: update CAN message send to use new library


  float temp = sht20.readTemperature();
  float humidity = sht20.readHumidity() / 100.0;

  // Update this
  CAN.sendMsgBuf((0x10 << 8) | (0x60 << 1), 1, 8, stmp);
  delay(10);
  CAN.sendMsgBuf((0x10 << 8) | (0x60 << 1), 1, 8, stmp);
  delay(500);                       // send data per 100ms
  Serial.println("CAN BUS sendMsgBuf ok!");

  delay(1000);
}
