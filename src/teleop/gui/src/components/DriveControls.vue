<template>
    <div>
      <!-- <div class="controls">
        <Checkbox ref="teleop-enabled" v-bind:name="'Teleop Drive Enabled'" v-on:toggle="teleopEnabled = $event"/>
      </div> -->
    </div>
</template>

<script>

import ROSLIB from "roslib"
import Checkbox from "./Checkbox.vue"

let interval;

export default {
  data () {
    return {
      joystick_pub: null
    }
  },
  
  
  beforeDestroy: function () {
    window.clearInterval(interval);
  },
  
  created: function () {
    const updateRate = 0.05;
    this.joystick_pub = new ROSLIB.Topic({
      ros : this.$ros,
      name : '/joystick',
      messageType : 'sensor_msgs/Joy'
    })
    interval = window.setInterval(() => {
        const gamepads = navigator.getGamepads()
        for (let i = 0; i < 4; i++) {
          const gamepad = gamepads[i]
          if (gamepad) {
            if (gamepad.id.includes('Logitech')) {
            
              let buttons = gamepad.buttons.map((button) =>{
                return button.value
              })

              const joystickData = {
                axes: gamepad.axes,
                buttons: buttons
              }
              
              var joystickMsg = new ROSLIB.Message(joystickData)
              this.joystick_pub.publish(joystickMsg)
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