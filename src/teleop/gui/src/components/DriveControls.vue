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
            
              let buttons = gamepad.buttons.map((button) =>{
                return button.value
              })

              //Deadzone applied to all axis
              const deadzone = 0.05
              let axes = gamepad.axes.map((axis) => {
                return Math.abs(axis) <= deadzone ? 0 : axis
              })
              //Invert Forward/Back Stick, forward is normally -1
              axes[JOYSTICK_CONFIG['forward_back']] = -1 * axes[JOYSTICK_CONFIG['forward_back']]

              //Done so that teleop_twist_joy will always be enabled
              buttons.unshift(1)

              const joystickData = {
                axes: axes,
                buttons: buttons
              }
              
              var joystickTopic = new ROSLIB.Topic({
                ros : this.$ros,
                name : '/joystick',
                messageType : 'sensor_msgs/Joy'
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