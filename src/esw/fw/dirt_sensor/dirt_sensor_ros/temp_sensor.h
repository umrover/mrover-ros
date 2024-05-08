#pragma once
constexpr static float RESISTANCE_25 = 10000.0f;
constexpr static float BIAS_RESISTOR = 984.0f;  // diagram is ground - thermistor - analog input - bias resistor - 3.3v
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
      float measured_resistance = (measured_voltage * BIAS_RESISTOR)/(3.3 - measured_voltage);
      float temp = 1/(THRM_A + THRM_B*log(measured_resistance/RESISTANCE_25) + THRM_C*log((measured_resistance/RESISTANCE_25)*(measured_resistance/RESISTANCE_25)) + THRM_D*(((measured_resistance/RESISTANCE_25)*(measured_resistance/RESISTANCE_25)*(measured_resistance/RESISTANCE_25))));
      temp -= 273.15;
      return temp;
    }

    int getRawData() {
      return analogRead(TEMP_SENSOR_PIN);
    }

    float getVoltage() {
      int rawData = getRawData();
      float magic_number = 3.3 / 3.54; // added because there is clearly error
      float voltage = rawData * 5.0 / 1024.0 * magic_number;
      return voltage;
    }
  private:
};
