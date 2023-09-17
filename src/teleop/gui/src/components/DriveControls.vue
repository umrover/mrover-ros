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
            if (gamepad.id.includes('Logitech')) {
            
              let buttons = gamepad.buttons.map((button) =>{
                return button.value
              })

              let axes=gamepad.axes

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