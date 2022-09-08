#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   Connections
   ===========
   Connect SCL to SCL pin (analog 5 on Arduino UNO)
   Connect SDA to SDA pin (analog 4 on Arduino UNO)
   Connect VIN to 3-5V DC (depending on your board's logic level)
   Connect GND to common ground
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define I2C_ADDRESS 0x28
#define SENSOR_ID 55

#define G 9.81

Adafruit_BNO055 bno = Adafruit_BNO055(SENSOR_ID, I2C_ADDRESS);

void setup(void)
{
  Serial.begin(115200);

  // init the IMU
  if (!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);

  // use the external crystal clock for better accuracy
  bno.setExtCrystalUse(true);
}

void loop(void)
{
  // get orientation, acceleration, and gyroscope data,
  // each from their own sensor event
  sensors_event_t accelData, gyroData, magData;
  imu::Quaternion orientationQuaternion;

  orientationQuaternion = bno.getQuat();
  bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // get calibration status for each sensor
  uint8_t system_cal, gyro_cal, accel_cal, mag_cal;
  system_cal = gyro_cal = accel_cal = mag_cal = 0;
  bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);

  // send the floating point data over serial in the following format:
  // accel_x accel_y accel_z gyro_x gyro_y gyro_z bearing sys_calibration gyro_calibration accel_calibration mag_calibration
  Serial.print(accelData.acceleration.x / G, 4);
  Serial.print(" ");
  Serial.print(accelData.acceleration.y / G, 4);
  Serial.print(" ");
  Serial.print(accelData.acceleration.z / G, 4);

  Serial.print(" ");
  Serial.print(gyroData.gyro.x * (180/PI), 4);
  Serial.print(" ");
  Serial.print(gyroData.gyro.y * (180/PI), 4);
  Serial.print(" ");
  Serial.print(gyroData.gyro.z * (180/PI), 4);

  Serial.print(" ");
  Serial.print(magData.magnetic.x, 4);
  Serial.print(" ");
  Serial.print(magData.magnetic.y, 4);
  Serial.print(" ");
  Serial.print(magData.magnetic.z, 4);

  Serial.print(" ");
  Serial.print(system_cal, DEC);
  Serial.print(" ");
  Serial.print(gyro_cal, DEC);
  Serial.print(" ");
  Serial.print(accel_cal, DEC);
  Serial.print(" ");
  Serial.print(mag_cal, DEC);

  // print the bearing around the axis perpendicular to the board,
  // for some reason it is labeled as the x axis for orientation data,
  // should be the Z axis
  Serial.print(" ");
  Serial.println(orientationData.orientation.x, 4);

  // wait the specified delay before requesting nex data
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
