<template>
  <div class="wrap">
    <h3> Arm controls </h3>
    <div class="controls">
      <Checkbox ref="arm-enabled" v-bind:name="'Arm Enabled'" v-on:toggle="updateArmEnabled($event)"/>
      <Checkbox ref="A" v-bind:name="'A'" v-on:toggle="updateArmEnabled($event)"/>
      <Checkbox ref="B" v-bind:name="'B'" v-on:toggle="updateArmEnabled($event)"/>
      <Checkbox ref="C" v-bind:name="'C'" v-on:toggle="updateArmEnabled($event)"/>
      <Checkbox ref="D" v-bind:name="'D'" v-on:toggle="updateArmEnabled($event)"/>
      <Checkbox ref="E" v-bind:name="'E'" v-on:toggle="updateArmEnabled($event)"/>
      <Checkbox ref="F" v-bind:name="'F'" v-on:toggle="updateArmEnabled($event)"/>
      
    </div>
  </div>
</template>

<script>
import ROSLIB from 'roslib'
import Checkbox from './Checkbox.vue'
import { mapGetters, mapMutations } from 'vuex'


let interval;

export default {
  data() {
    return {
      arm_enabled: false,
      joystick_pub: null,
      joints_array: [false, false, false,false,false, false]
    }
  },

  computed: {

    ...mapGetters('controls', {
      controlMode: 'controlMode'
    }),
  },

  beforeDestroy: function () {
    window.clearInterval(interval)
  },


  created: function () {
    this.joystick_pub = new ROSLIB.Topic({
        ros : this.$ros,
        name : '/xbox/ra_control',
        messageType : 'sensor_msgs/Joy'
    })
    const updateRate = 0.1
    interval = window.setInterval(() => {
      if(this.arm_enabled){
        const gamepads = navigator.getGamepads()
        for (let i = 0; i < 4; i++) {
          const gamepad = gamepads[i]
          if (gamepad) {
            if (gamepad.id.includes('Microsoft') || gamepad.id.includes('Xbox')) {
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
      }
    }, updateRate*1000)
  },

    methods: {
        updateArmEnabled: function (enabled){
            this.arm_enabled = enabled
        },
        updateJointsEnabled: function(enabled){

        }
    },

    components: {
        Checkbox
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