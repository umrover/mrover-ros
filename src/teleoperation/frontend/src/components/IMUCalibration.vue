<template>
  <div class="rounded bg-white wrap">
    <span class="px-2">IMU Calibration: {{ calibration }}</span>
    <LEDIndicator
      class="px-2"
      :show_name="false"
      :connected="calibration == calibration_limit_master"
    />
    <span class="px-2">IMU Temperature: {{ temperature }} °C</span>
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
      calibration: 0,
      temperature: 0,
      calibration_limit_master: calibration_limit
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      switch (msg.type) {
        case 'temperature':
          this.temperature = msg.temperature
          break
        case 'calibration_status':
          this.calibration = msg.system_calibration
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
