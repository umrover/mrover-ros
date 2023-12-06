<template>
  <div class="wrap">
    <div class="table">
      <table class="tableFormat" style="undefined;table-layout:fixed;border=0">
        <caption>
          IMU Calibration
        </caption>
        <colgroup>
          <col style="width: 80px" />
          <col style="width: 80px" />
          <col style="width: 80px" />
          <col style="width: 80px" />
        </colgroup>
        <thead>
          <tr>
            <td class="tableElement">Mag</td>
            <td class="tableElement">Accel</td>
            <td class="tableElement">Gyro</td>
            <td class="tableElement">System</td>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>
              <span
                :class="[
                  'led',
                  magnetometer_val == calibration_limit_master ? 'green' : 'red'
                ]"
              ></span>
            </td>
            <td>
              <span
                :class="[
                  'led',
                  accelerometer_val == calibration_limit_master
                    ? 'green'
                    : 'red'
                ]"
              ></span>
            </td>
            <td>
              <span
                :class="[
                  'led',
                  gyroscope_val == calibration_limit_master ? 'green' : 'red'
                ]"
              ></span>
            </td>
            <td>
              <span
                :class="[
                  'led',
                  system_val == calibration_limit_master ? 'green' : 'red'
                ]"
              ></span>
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
  </div>
</template>

<script lang="ts">
import { mapState } from 'vuex';

const calibration_limit = 3;

export default {
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
.led {
  width: 16px;
  height: 16px;
  border-radius: 8px;
  border: 1px solid;
  display: block;
}
.green {
  background-color: lightgreen;
}
.red {
  background-color: red;
}
.numbers {
  padding-left: 10px;
}
</style>