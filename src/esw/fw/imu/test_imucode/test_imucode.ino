#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <stdio.h>
#include "canbed_dual.h"

#define BNO08X_RESET -1

union BytesToFloats {
    unsigned char raw_data[48];
    struct Floats {
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

        float trash[2];
    } imu_data;
};

BytesToFloats buffer;

CANBedDual CAN0(0);
CANBedDual CAN1(1);

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setup()
{
    Serial.begin(115200);
    // CAN Setup
    pinMode(18, OUTPUT);
    Wire1.setSDA(6);
    Wire1.setSCL(7);
    Wire1.begin();
    CAN0.initFD(1000000, 1000000);          // CAN0 baudrate: 1Mb/s, FD baudrate, 1Mb/s

    // IMU Setup
    Wire.setSDA(12);
    Wire.setSCL(13);
    if (!bno08x.begin_I2C()) {
        Serial.println("Failed to find BNO08x chip");
        while (1) {
        delay(10);
        }
    }
    setReports();
    buffer.imu_data.trash[0] = 0;
    buffer.imu_data.trash[1] = 0;
}

void loop()
{
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
            buffer.imu_data.quaternion_x = sensorValue.un.rotationVector.i;
            buffer.imu_data.quaternion_y = sensorValue.un.rotationVector.j;
            buffer.imu_data.quaternion_z = sensorValue.un.rotationVector.k;
            buffer.imu_data.quaternion_w = sensorValue.un.rotationVector.real;
            break;

        case SH2_LINEAR_ACCELERATION:
            // Serial.print("LINEAR ACCELERATION ");
            //acceleration
            buffer.imu_data.linearAcceleration_x = sensorValue.un.linearAcceleration.x;
            buffer.imu_data.linearAcceleration_y = sensorValue.un.linearAcceleration.y;
            buffer.imu_data.linearAcceleration_z = sensorValue.un.linearAcceleration.z;
            break;

        case SH2_GYROSCOPE_CALIBRATED:
            // Serial.print("GYRO  ");
            //gyro
            buffer.imu_data.gyroscope_x = sensorValue.un.gyroscope.x;
            buffer.imu_data.gyroscope_y = sensorValue.un.gyroscope.y;
            buffer.imu_data.gyroscope_z = sensorValue.un.gyroscope.z;
            break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
            // Serial.print("MAG ");
            // orientation
            Serial.print(buffer.imu_data.quaternion_x, 6);
            Serial.print(F(" "));
            Serial.print(buffer.imu_data.quaternion_y, 6);
            Serial.print(F(" "));
            Serial.print(buffer.imu_data.quaternion_z, 6);
            Serial.print(F(" "));
            Serial.print(buffer.imu_data.quaternion_w, 6);
            Serial.print(F(" "));

            //acceleration
            Serial.print(buffer.imu_data.linearAcceleration_x, 6);
            Serial.print(F(" "));
            Serial.print(buffer.imu_data.linearAcceleration_y, 6);
            Serial.print(F(" "));
            Serial.print(buffer.imu_data.linearAcceleration_z, 6);
            Serial.print(F(" "));

            //gyro
            Serial.print(buffer.imu_data.gyroscope_x, 6);
            Serial.print(F(" "));
            Serial.print(buffer.imu_data.gyroscope_y, 6);
            Serial.print(F(" "));
            Serial.print(buffer.imu_data.gyroscope_z, 6);
            Serial.print(F(" "));

            //mag
            Serial.print(sensorValue.un.magneticField.x, 6);
            Serial.print(F(" "));
            Serial.print(sensorValue.un.magneticField.y, 6);
            Serial.print(F(" "));
            Serial.print(sensorValue.un.magneticField.z, 6);
            Serial.print(F(" "));
            Serial.print(sensorValue.un.temperature.value);
            Serial.print(F(" "));

            // Serial.print("Calibration ");
            Serial.print(sensorValue.status);
            Serial.print(F("\n"));
            break;
    }
    CAN0.send(0x01, 0, 0, 1, 48, buffer.raw_data);
}

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