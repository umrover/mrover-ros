<template>
  <div class="wrap">
    <h3> Arm controls </h3>
    <div class="controls">
      <RoverCheckbox ref="arm-enabled" v-bind:name="'Arm Enabled'" v-on:toggle="updateArmEnabled($event)"/>
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import RoverCheckbox from './RoverCheckbox.vue'

let interval

export default {
  data () {
    return {
      arm_enabled: false
    }
  },

  beforeUnmount: function () {
    window.clearInterval(interval)
  },

  created: function () {
    const updateRate = 0.1
    interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      for (let i = 0; i < 4; i++) {
        const gamepad = gamepads[i]
        if (gamepad && this.arm_enabled) {
          if (gamepad.id.includes('Microsoft') || gamepad.id.includes('Xbox')) {
            const buttons = gamepad.buttons.map((button) => {
              return button.value
            })

            const axes = gamepad.axes

            const joystickData = {
              axes,
              buttons
            }
            const joystickTopic = new ROSLIB.Topic({
              ros: this.$ros,
              name: '/xbox/ra_control',
              messageType: 'sensor_msgs/Joy'
            })
            const joystickMsg = new ROSLIB.Message(joystickData)
            joystickTopic.publish(joystickMsg)
          }
        }
      }
    }, updateRate * 1000)
  },

  methods: {
    updateArmEnabled: function (enabled) {
      this.arm_enabled = enabled
    }
  },

  components: {
    RoverCheckbox
  }
}
</script>

<style scoped>

.wrap {
  display: inline-block;
  align-items: center;
  justify-items: center;
  width: 100%;
}
.controls {
  display: flex;
  align-items:center;
}
.header {
  display:flex;
  align-items:center;
}
.joint-b-calibration {
  display: flex;
  gap: 10px;
  width: 250px;
  font-weight: bold;
  color: red;
}

</style>
