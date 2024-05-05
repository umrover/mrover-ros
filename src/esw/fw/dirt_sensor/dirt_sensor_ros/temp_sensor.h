#pragma once

constexpr static float DIAG_TEMP_COEFFICIENT = 0.0064f;
constexpr static float DIAG_TEMP_25_DEGREE_RESISTANCE = 10000.0;
constexpr static float THRM_A0 = -5.160732E+02;
constexpr static float THRM_A1 = 6.831122E+02;
constexpr static float THRM_A2 = -3.774928E+02;
constexpr static float THRM_A3 = 1.159826E+02;
constexpr static float THRM_A4 = -1.060407E+01;

constexpr static float RESISTANCE_25 = 10000.0;
constexpr static float THRM_A = 3.3540170E-03;
constexpr static float THRM_B = 2.5617244E-04;
constexpr static float THRM_C = 2.1400943E-06;
constexpr static float THRM_D = -7.2405219E-08;

#define TEMP_SENSOR_PIN A1

class TempSensor {
  public:
    void setup() {
      pinMode(TEMP_SENSOR_PIN, INPUT); 
    }

    float getTemperature() {
      float measured_voltage = getVoltage();
      float measured_resistance = (measured_voltage * RESISTANCE_25)/(3.3 - measured_voltage);
      float temp = 1/(THRM_A + THRM_B*log(measured_resistance/RESISTANCE_25) + THRM_C*log((measured_resistance/RESISTANCE_25)*(measured_resistance/RESISTANCE_25)) + THRM_D*(((measured_resistance/RESISTANCE_25)*(measured_resistance/RESISTANCE_25)*(measured_resistance/RESISTANCE_25))));
      temp -= 273.15;
//      float temp = (THRM_A4 * powf(measured_voltage,4)) + (THRM_A3 * powf(measured_voltage,3)) + (THRM_A2 * powf(measured_voltage,2)) + (THRM_A1 *  measured_voltage) + THRM_A0;
      return temp;
    }

    int getRawData() {
      return analogRead(TEMP_SENSOR_PIN);
    }
  private:
    

    float getVoltage() {
      int rawData = getRawData();
      float voltage = rawData * 3.3 / 1024.0;
      return voltage;
    }
};
