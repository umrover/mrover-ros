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

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
    // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable magnetic field calibrated");
  }
}
void printActivity(uint8_t activity_id) {
  switch (activity_id) {
  case PAC_UNKNOWN:
    Serial.print("Unknown");
    break;
  case PAC_IN_VEHICLE:
    Serial.print("In Vehicle");
    break;
  case PAC_ON_BICYCLE:
    Serial.print("On Bicycle");
    break;
  case PAC_ON_FOOT:
    Serial.print("On Foot");
    break;
  case PAC_STILL:
    Serial.print("Still");
    break;
  case PAC_TILTING:
    Serial.print("Tilting");
    break;
  case PAC_WALKING:
    Serial.print("Walking");
    break;
  case PAC_RUNNING:
    Serial.print("Running");
    break;
  case PAC_ON_STAIRS:
    Serial.print("On Stairs");
    break;
  default:
    Serial.print("NOT LISTED");
  }
  Serial.print(" (");
  Serial.print(activity_id);
  Serial.print(")");
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
    //orientation
    Serial.print(sensorValue.un.rotationVector.i);
    Serial.print(" ");
    Serial.print(sensorValue.un.rotationVector.j);
    Serial.print(" ");
    Serial.print(sensorValue.un.rotationVector.k);
    Serial.print(" ");
    Serial.print(sensorValue.un.rotationVector.real);
    Serial.print(" ");

    //acceleration
    Serial.print(sensorValue.un.accelerometer.x, 6);
    Serial.print(" ");
    Serial.print(sensorValue.un.accelerometer.y, 6);
    Serial.print(" ");
    Serial.print(sensorValue.un.accelerometer.z, 6);
    Serial.print(" ");

    //gyro
    Serial.print(sensorValue.un.gyroscope.x, 6);
    Serial.print(" ");
    Serial.print(sensorValue.un.gyroscope.y, 6);
    Serial.print(" ");
    Serial.print(sensorValue.un.gyroscope.z, 6);
    Serial.print(" ");

    //mag
    Serial.print(sensorValue.un.magneticField.x, 6);
    Serial.print(" ");
    Serial.print(sensorValue.un.magneticField.y, 6);
    Serial.print(" ");
    Serial.print(sensorValue.un.magneticField.z, 6);
    Serial.print(" ");

    Serial.print(sensorValue.status); 
    Serial.println();
}

