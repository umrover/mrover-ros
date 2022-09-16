<template>
    <div>
        <h4>Drive Controls</h4>
    </div>
</template>

<script>

import ROSLIB from "roslib"

let interval;

export default {
  data () {
    return {
    }
  },


  beforeDestroy: function () {
    window.clearInterval(interval);
  },

  created: function () {
    const updateRate = 0.05;
    interval = window.setInterval(() => {
        const gamepads = navigator.getGamepads()
        for (let i = 0; i < 4; i++) {
          const gamepad = gamepads[i]
          if (gamepad) {
            if (gamepad.id.includes('Xbox')) {
            
              let buttons = gamepad.buttons.map((button) =>{
                return button.value
              })

<<<<<<< HEAD
<<<<<<< HEAD
              let axes=gamepad.axes
=======
              //Deadzone applied to all axis
              const deadzone = 0.05
              let axes = gamepad.axes.map((axis) => {
                return Math.abs(axis) <= deadzone ? 0 : axis
              })
              
              //Invert Forward/Back Stick, forward is normally -1
              axes[this.joystick_mapping.forward_back] = this.drive_config.forward_back.multiplier * axes[this.joystick_mapping.forward_back]
              axes[this.joystick_mapping.left_right] = this.drive_config.left_right.multiplier * axes[this.joystick_mapping.left_right]
>>>>>>> Changed wheel control interface back to [-1,1]
=======
              let axes=gamepad.axes
>>>>>>> Moved axis multiplier logic to jetson_teleop

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