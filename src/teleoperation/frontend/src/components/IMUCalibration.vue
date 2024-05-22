<template>
  <div class="rounded bg-white d-flex flex-row align-items-center">
    <span class="px-2">IMU Calibration</span>
    <span class="px-2">Magnetometer</span>
    <LEDIndicator
      class="px-2"
      :name="mag_calibration.toString()"
      :show_name="true"
      :connected="mag_calibration == calibration_limit_master"
    />
    <span class="px-2">Gyroscope</span>
    <LEDIndicator
      class="px-2"
      :name="gyro_calibration.toString()"
      :show_name="true"
      :connected="gyro_calibration == calibration_limit_master"
    />
    <span class="px-2">Accelerometer</span>
    <LEDIndicator
      class="px-2"
      :name="accel_calibration.toString()"
      :show_name="true"
      :connected="accel_calibration == calibration_limit_master"
    />
  </div>
</template>

<script lang="ts">
import { mapState } from 'vuex'
import LEDIndicator from './LEDIndicator.vue'

const calibration_limit = 3

export default {
  components: {
    LEDIndicator
  },

  data() {
    return {
      mag_calibration: 0,
      gyro_calibration: 0,
      accel_calibration: 0,
      calibration_limit_master: calibration_limit
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      switch (msg.type) {
        case 'calibration':
          this.mag_calibration = msg.magnetometer_calibration
          this.gyro_calibration = msg.gyroscope_calibration
          this.accel_calibration = msg.acceleration_calibration
          break
      }
    }
  }
}
</script>
<style scoped>
.wrap {
  display: flex;
  justify-content: center;
  height: 100%;
}
</style>
