#define USE_USBCON

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

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

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define I2C_ADDRESS 0x28
#define SENSOR_ID 55
#define G 9.81

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
sensor_msgs::Temperature temp_msg;

ros::Publisher imu_pub("imu", &imu_msg);
ros::Publisher mag_pub("magnetometer", &mag_msg);
ros::Publisher temp_pub("imu_temp", &temp_msg);

Adafruit_BNO055 bno_imu = Adafruit_BNO055(SENSOR_ID, I2C_ADDRESS);

void setup()
{
  // init ROS serial node
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(mag_pub);
  nh.advertise(temp_pub);

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
  bno_imu.getEvent(&accel_data, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno_imu.getEvent(&gyro_data, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno_imu.getEvent(&mag_data, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // get calibration status for each sensor
  uint8_t system_cal, gyro_cal, accel_cal, mag_cal;
  system_cal = gyro_cal = accel_cal = mag_cal = 0;
  bno_imu.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);

  // populate IMU ROS message
  imu_msg.orientation.x = orientation_quat.x();
  imu_msg.orientation.y = orientation_quat.y();
  imu_msg.orientation.z = orientation_quat.z();
  imu_msg.orientation.w = orientation_quat.w();

  imu_msg.linear_acceleration.x = accel_data.acceleration.x;
  imu_msg.linear_acceleration.y = accel_data.acceleration.y;
  imu_msg.linear_acceleration.z = accel_data.acceleration.z;

  imu_msg.angular_velocity.x = gyro_data.gyro.x;
  imu_msg.angular_velocity.y = gyro_data.gyro.y;
  imu_msg.angular_velocity.z = gyro_data.gyro.z;

  // TODO: fill in convariance

  // populate magnetometer ROS message
  mag_msg.magnetic_field.x = mag_data.magnetic.x;  
  mag_msg.magnetic_field.y = mag_data.magnetic.y;  
  mag_msg.magnetic_field.z = mag_data.magnetic.z;  

  // TODO: fill in covariance

  temp_msg.temperature = bno_imu.getTemp();

  // TODO: create custom calibration status message and publish it
  // Serial.print(system_cal, DEC);
  // Serial.print(gyro_cal, DEC);
  // Serial.print(accel_cal, DEC);
  // Serial.print(mag_cal, DEC);

  // TODO: find out what getSystemStatus() does

  // publish IMU data
  imu_pub.publish(&imu_msg);
  mag_pub.publish(&mag_msg);
  temp_pub.publish(&temp_msg);
  nh.spinOnce();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
