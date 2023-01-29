<template>
  <div class="wrap">
    <h4> Drive </h4>
    <div class="controls">
      <span>Speed Limiter: {{ dampenDisplay }}%</span>
      <Checkbox class="reverse" ref="reverse" v-bind:name="'Reverse'" v-on:toggle="updateReverse($event)"/>
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
      joystick_pub: null,
      dampenDisplay: 0
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
