<template>
    <div class="wrap">
      <table class="table">
        <caption>
          IMU Calibration
        </caption>
        <thead>
          <tr>
            <td scope="col">Mag</td>
            <td scope="col">Accel</td>
            <td scope="col">Gyro</td>
            <td scope="col">System</td>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>
              <LEDIndicator :connected=" magnetometer_val == calibration_limit_master"/>
            </td>
            <td>
              <LEDIndicator :connected="accelerometer_val == calibration_limit_master"/>
            </td>
            <td>
              <LEDIndicator :connected="gyroscope_val == calibration_limit_master"/>
            </td>
            <td>
              <LEDIndicator :connected="system_val == calibration_limit_master"/>
            </td>
          </tr>
          <tr>
            <td class="numbers">{{ magnetometer_val }}</td>
            <td class="numbers">{{ accelerometer_val }}</td>
            <td class="numbers">{{ gyroscope_val }}</td>
            <td class="numbers">{{ system_val }}</td>
          </tr>
        </tbody>
        <tbody></tbody>
      </table>
    </div>
  </template>
  
  <script lang="ts">
  import { mapState } from 'vuex';
  import LEDIndicator from './LEDIndicator.vue';
  
  const calibration_limit = 3;
  
  export default {
    components : {
        LEDIndicator
    },
    
    data() {
      return {
        system_val: 0,
        gyroscope_val: 0,
        accelerometer_val: 0,
        magnetometer_val: 0,
        calibration_limit_master: calibration_limit,
      };
    },
    
    computed: {
      ...mapState('websocket', ['message'])
    },
  
    watch: {
      message(msg) {
        if (msg.type == "calibration_status") {
          this.system_val = msg.system_calibration;
          this.gyroscope_val = msg.gyroscope_calibration;
          this.accelerometer_val = msg.accelerometer_calibration;
          this.magnetometer_val = msg.magnetometer_calibration;
        }
      }
    }
  };
  </script>
  <style scoped>
  .wrap {
    display: flex;
    align-items: center;
    height: 100%;
  }
  /* .numbers {
    padding-left: 10px;
  } */
  </style>