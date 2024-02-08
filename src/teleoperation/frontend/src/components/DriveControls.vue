<template>
  <div class="drive">
    <!-- This component is for capturing joystick inputs -->
  </div>
</template>

<script lang="ts">
import { mapActions } from 'vuex'

let interval: number

export default {
  data() {
    return {}
  },

  methods: {
    ...mapActions('websocket', ['sendMessage'])
  },

  beforeUnmount: function () {
    window.clearInterval(interval)
  },

  created: function () {
    const UPDATE_RATE = 10; //100 Hz

    interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      for (let i = 0; i < 4; i++) {
        const gamepad = gamepads[i]
        if (gamepad && (gamepad.id.includes('Logitech') || gamepad.id.includes('Thrustmaster'))) {
          let buttons = gamepad.buttons.map((button) => {
            return button.value
          })

          const joystickData = {
            type: 'joystick_values',
            axes: gamepad.axes,
            buttons: buttons
          }

          this.sendMessage(joystickData)
        }
      }
    }, UPDATE_RATE)
  }
}
</script>

<style scoped>
.drive {
  display: none;
}
</style>
