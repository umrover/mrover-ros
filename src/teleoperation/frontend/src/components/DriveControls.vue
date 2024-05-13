<template>
  <div class='drive' />
</template>

<script lang='ts'>
import { mapActions } from 'vuex'

const UPDATE_HZ = 20

export default {
  methods: {
    ...mapActions('websocket', ['sendMessage'])
  },

  beforeUnmount: function() {
    window.clearInterval(this.interval)
  },

  created: function() {
    this.interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      const gamepad = gamepads.find(gamepad => gamepad && gamepad.id.includes('Thrustmaster'))
      if (!gamepad) return

      this.sendMessage({
        type: 'joystick',
        axes: gamepad.axes,
        buttons: gamepad.buttons.map(button => button.value)
      })
    }, 1000 / UPDATE_HZ)
  }
}
</script>

<style scoped>
.drive {
  display: none;
}
</style>
