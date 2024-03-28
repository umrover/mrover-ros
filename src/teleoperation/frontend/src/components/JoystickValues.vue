<template>
  <div class="datagrid">
    <div>
      <p style="margin-top: 0px">left_right: {{ joystick_values.left_right.toFixed(3) }}</p>
      <p>forward_back: {{ joystick_values.forward_back.toFixed(3) }}</p>
      <p>twist: {{ joystick_values.twist.toFixed(3) }}</p>
    </div>
    <div>
      <p style="margin-top: 0px">dampen: {{ joystick_values.dampen.toFixed(3) }}</p>
      <p>pan: {{ joystick_values.pan.toFixed(3) }}</p>
      <p>tilt: {{ joystick_values.tilt.toFixed(3) }}</p>
    </div>
  </div>
</template>

<script lang="ts">
import { mapState } from 'vuex'

interface JoystickValues {
  left_right: number
  forward_back: number
  twist: number
  dampen: number
  pan: number
  tilt: number
}

export default {
  data() {
    return {
      joystick_mappings: {},
      joystick_values: {
        left_right: 0,
        forward_back: 0,
        twist: 0,
        dampen: 0,
        pan: 0,
        tilt: 0
      } as JoystickValues
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == 'joystick') {
        this.joystick_values.left_right = msg.left_right
        this.joystick_values.forward_back = msg.forward_back
        this.joystick_values.twist = msg.twist
        this.joystick_values.dampen = msg.dampen
        this.joystick_values.pan = msg.pan
        this.joystick_values.tilt = msg.tilt
      }
    }
  }
}
</script>
<style scoped>
.datagrid {
  display: grid;
  grid-template-columns: auto auto;
  gap: 10px;
  line-height: 0.5em;
}
</style>
