#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

float quaternion_x;
float quaternion_y;
float quaternion_z;
float quaternion_w;

float linearAcceleration_x;
float linearAcceleration_y;
float linearAcceleration_z;

float gyroscope_x;
float gyroscope_y;
float gyroscope_z;


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
    // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }

  setReports();
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println("Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000)) {
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000)) {
    Serial.println("Could not enable magnetic field calibrated");
  }
}


void loop() {
  delay(10);

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  switch (sensorValue.sensorId) {

  case SH2_ROTATION_VECTOR:
    // Serial.print("ORIENTATION ");
    // orientation
    quaternion_x = sensorValue.un.rotationVector.i;
    quaternion_y = sensorValue.un.rotationVector.j;
    quaternion_z = sensorValue.un.rotationVector.k;
    quaternion_w = sensorValue.un.rotationVector.real;
    break;

  case SH2_LINEAR_ACCELERATION:
    // Serial.print("LINEAR ACCELERATION ");
    //acceleration
    linearAcceleration_x = sensorValue.un.linearAcceleration.x;
    linearAcceleration_y = sensorValue.un.linearAcceleration.y;
    linearAcceleration_z = sensorValue.un.linearAcceleration.z;
    break;

  case SH2_GYROSCOPE_CALIBRATED:
    // Serial.print("GYRO  ");
    //gyro
    gyroscope_x = sensorValue.un.gyroscope.x;
    gyroscope_y = sensorValue.un.gyroscope.y;
    gyroscope_z = sensorValue.un.gyroscope.z;
    break;

  case SH2_MAGNETIC_FIELD_CALIBRATED:
    // Serial.print("MAG ");
    // orientation
    Serial.print(quaternion_x, 6);
    Serial.print(" ");
    Serial.print(quaternion_y, 6);
    Serial.print(" ");
    Serial.print(quaternion_z, 6);
    Serial.print(" ");
    Serial.print(quaternion_w, 6);
    Serial.print(" ");

    //acceleration
    Serial.print(linearAcceleration_x, 6);
    Serial.print(" ");
    Serial.print(linearAcceleration_y, 6);
    Serial.print(" ");
    Serial.print(linearAcceleration_z, 6);
    Serial.print(" ");

    //gyro
    Serial.print(gyroscope_x, 6);
    Serial.print(" ");
    Serial.print(gyroscope_y, 6);
    Serial.print(" ");
    Serial.print(gyroscope_z, 6);
    Serial.print(" ");

    //mag
    Serial.print(sensorValue.un.magneticField.x, 6);
    Serial.print(" ");
    Serial.print(sensorValue.un.magneticField.y, 6);
    Serial.print(" ");
    Serial.print(sensorValue.un.magneticField.z, 6);
    Serial.print(" ");
    Serial.print(sensorValue.un.temperature.value); 
    Serial.print(" ");

    // Serial.print("Calibration ");
    Serial.print(sensorValue.status); 
    Serial.print("\n");
    break;

  }

}