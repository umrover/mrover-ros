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

#define BNO055_SAMPLERATE_DELAY_MS (50)
#define I2C_ADDRESS 0x28
#define SENSOR_ID 55
#define G 9.81

Adafruit_BNO055 bno_imu = Adafruit_BNO055(SENSOR_ID, I2C_ADDRESS);

void setup()
{
  Serial.begin(115200);
  // init the IMU, defaults to NDOF mode
  if (!bno_imu.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);

  // use the external crystal clock for better accuracy
  bno_imu.setExtCrystalUse(true);
}

void loop()
{
  // get orientation, acceleration, and gyroscope data,
  // each from their own sensor event
  sensors_event_t accel_data, gyro_data, mag_data;
  imu::Quaternion orientation_quat;

  orientation_quat = bno_imu.getQuat();

  // not using "linear acceleration" because we want accel from gravity to be included
  bno_imu.getEvent(&accel_data, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno_imu.getEvent(&gyro_data, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno_imu.getEvent(&mag_data, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // get calibration status for each sensor
  uint8_t system_cal, gyro_cal, accel_cal, mag_cal;
  system_cal = gyro_cal = accel_cal = mag_cal = 0;
  bno_imu.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);

  // imu_msg.orientation.x = orientation_quat.x();
  // imu_msg.orientation.y = orientation_quat.y();
  // imu_msg.orientation.z = orientation_quat.z();
  // imu_msg.orientation.w = orientation_quat.w();

  Serial.print(orientation_quat.x(), 6);
  Serial.print(" ");
  Serial.print(orientation_quat.y(), 6);
  Serial.print(" ");
  Serial.print(orientation_quat.z(), 6);
  Serial.print(" ");
  Serial.print(orientation_quat.w(), 6);
  Serial.print(" ");

  // imu_msg.linear_acceleration.x = accel_data.acceleration.x;
  // imu_msg.linear_acceleration.y = accel_data.acceleration.y;
  // imu_msg.linear_acceleration.z = accel_data.acceleration.z;

  Serial.print(accel_data.acceleration.x, 6);
  Serial.print(" ");
  Serial.print(accel_data.acceleration.y, 6);
  Serial.print(" ");
  Serial.print(accel_data.acceleration.z, 6);
  Serial.print(" ");

  // imu_msg.angular_velocity.x = gyro_data.gyro.x;
  // imu_msg.angular_velocity.y = gyro_data.gyro.y;
  // imu_msg.angular_velocity.z = gyro_data.gyro.z;

  Serial.print(gyro_data.gyro.x, 6);
  Serial.print(" ");
  Serial.print(gyro_data.gyro.y, 6);
  Serial.print(" ");
  Serial.print(gyro_data.gyro.z, 6);
  Serial.print(" ");

  // mag_msg.magnetic_field.x = mag_data.magnetic.x;  
  // mag_msg.magnetic_field.y = mag_data.magnetic.y;  
  // mag_msg.magnetic_field.z = mag_data.magnetic.z;  

  Serial.print(mag_data.magnetic.x, 6);
  Serial.print(" ");
  Serial.print(mag_data.magnetic.y, 6);
  Serial.print(" ");
  Serial.print(mag_data.magnetic.z, 6);
  Serial.print(" ");

  // temp_msg.temperature = bno_imu.getTemp();

  // TODO: put this in its own variable that is recorded at the same time as the others
  Serial.print(bno_imu.getTemp(), 6);
  Serial.print(" ");

  Serial.print(system_cal, DEC);
  Serial.print(" ");
  Serial.print(gyro_cal, DEC);
  Serial.print(" ");
  Serial.print(accel_cal, DEC);
  Serial.print(" ");
  Serial.print(mag_cal, DEC);
  Serial.println();

  // TODO: find out what getSystemStatus() does
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
