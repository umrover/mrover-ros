#include <ros.h>
#include <DFRobot_SHT20.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>
#include "temp_sensor.h"

ros::NodeHandle nh;

sensor_msgs::Temperature temperature_data;
ros::Publisher temperature_pub("sa_temp_data", &temperature_data);

sensor_msgs::Temperature thermistor_data;
ros::Publisher thermistor_pub("sa_thermistor_data", &thermistor_data);

sensor_msgs::RelativeHumidity humidity_data;
ros::Publisher humidity_pub("sa_humidity_data", &humidity_data);

DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);


TempSensor temp_sensor;

void setup(){
  nh.getHardware()->setBaud(57600); // Have to set this parameter
  nh.initNode();
  nh.advertise(temperature_pub);
  nh.advertise(thermistor_pub);
  nh.advertise(humidity_pub);
  //Serial.begin(9600);
  sht20.initSHT20();
  delay(100);
  temp_sensor.setup();
}

void loop(){

  float temp = sht20.readTemperature();
  temperature_data.temperature = temp;
  temperature_pub.publish(&temperature_data);

  float thermistorValue = temp_sensor.getTemperature(); 
  int rawVal = temp_sensor.getRawData();
  thermistor_data.temperature = thermistorValue;
  thermistor_pub.publish(&thermistor_data);
  // Serial.println(thermistorValue * 9/5 + 32);

  float humidity = sht20.readHumidity() / 100.0;
  humidity_data.relative_humidity = humidity;
  humidity_pub.publish(&humidity_data);

  nh.spinOnce();
  delay(1000);
}
