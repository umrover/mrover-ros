<template>
  <div class="rounded bg-white wrap">
    <span class="px-2">IMU Calibration</span>
    <LEDIndicator
      class="px-2"
      :name="magnetometer_calibration"
      :show_name="true"
      :connected="magnetometer_calibration == calibration_limit_master"
    />
    <LEDIndicator
      class="px-2"
      :name="gyroscope_calibration"
      :show_name="true"
      :connected="gyroscope_calibration == calibration_limit_master"
    />
    <LEDIndicator
      class="px-2"
      :name="acceleration_calibration"
      :show_name="true"
      :connected="acceleration_calibration == calibration_limit_master"
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
