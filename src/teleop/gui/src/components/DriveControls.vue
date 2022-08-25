<template>
    <div>
        <p>Drive Controls Present</p>
    </div>
</template>

<script>

import ROSLIB from "roslib"

let interval;

export default {
  data () {
    return {
      dampen: 0,
      reverse: false
    }
  },


  beforeDestroy: function () {
    window.clearInterval(interval);
  },

  created: function () {

    const JOYSTICK_CONFIG = {
      'left_right': 0,
      'forward_back': 1,
      'twist': 2,
      'dampen': 3,
      'pan': 4,
      'tilt': 5
    }


    const updateRate = 0.05;
    interval = window.setInterval(() => {
        const gamepads = navigator.getGamepads()
        for (let i = 0; i < 4; i++) {
          const gamepad = gamepads[i]
          if (gamepad) {
            if (gamepad.id.includes('Logitech')) {
              // Make dampen 1 when slider is pushed forward, 0 when pulled back
              // Raw value is -1 to 1
              this.dampen = gamepad.axes[JOYSTICK_CONFIG['dampen']] * -0.5 + 0.5
              // -1 multiplier to make turning left a positive value
              let rotation = -1 * (gamepad.axes[JOYSTICK_CONFIG['left_right']] + gamepad.axes[JOYSTICK_CONFIG['twist']])
              if (rotation > 1) {
                rotation = 1
              }
              else if (rotation < -1) {
                rotation = -1
              }
              const joystickData = {
                // forward on joystick is -1, so invert
                'forward_back': -1 * gamepad.axes[JOYSTICK_CONFIG['forward_back']],
                'left_right': rotation,
                'dampen': this.dampen
              }
              var joystickTopic = new ROSLIB.Topic({
                ros : this.$ros,
                name : '/joystick',
                messageType : 'mrover/Joystick'
              })
              var joystickMsg = new ROSLIB.Message(joystickData)
              joystickTopic.publish(joystickMsg)
            }
          }
        }
    }, updateRate*1000)
  },

}
</script>

<style scoped>

</style>