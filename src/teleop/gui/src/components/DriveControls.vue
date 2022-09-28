<template>
    <div>
      <div>
          <h4>Drive Controls</h4>
      </div>
      <div class="controls">
        <Checkbox ref="teleop-enabled" v-bind:name="'Teleop Drive Enabled'" v-on:toggle="teleopEnabled = $event"/>
      </div>
    </div>
</template>

<script>

import ROSLIB from "roslib"
import Checkbox from "./Checkbox.vue"

let interval;

export default {
  data () {
    return {
      teleopEnabled: false
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
          if (gamepad && this.teleopEnabled) {
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

  components:{
    Checkbox
  }

}
</script>

<style scoped>
.controls {
  display: flex;
  align-items:center;
}



</style>