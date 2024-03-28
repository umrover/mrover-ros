<template>
  <div class="rounded bg-white wrap">
    <span class="px-2">IMU Calibration</span>
    <span class="px-2">{{ system_val }}</span>
    <LEDIndicator
      class="px-2"
      :name="'System'"
      :show_name="true"
      :connected="system_val == calibration_limit_master"
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
      system_val: 0,
      calibration_limit_master: calibration_limit
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == 'calibration_status') {
        this.system_val = msg.system_calibration
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
