<template>
  <div class="wrap">
    <h3> Arm controls </h3>
    <div class="controls">
      <Checkbox ref="arm-enabled" v-bind:name="'Arm Enabled'" v-on:toggle="updateArmEnabled($event)"/>
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
      arm_enabled: false
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


    const XBOX_CONFIG = {
      'left_js_x': 0,
      'left_js_y': 1,
      'left_trigger': 6,
      'right_trigger': 7,
      'right_js_x': 2,
      'right_js_y': 3,
      'right_bumper': 5,
      'left_bumper': 4,
      'd_pad_up': 12,
      'd_pad_down': 13,
      'd_pad_right': 14,
      'd_pad_left': 15,
      'a': 0,
      'b': 1,
      'x': 2,
      'y': 3
    }


    const updateRate = 0.1
    interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      for (let i = 0; i < 4; i++) {
        const gamepad = gamepads[i]
        if (gamepad && this.arm_enabled) {
          if (gamepad.id.includes('Microsoft') || gamepad.id.includes('Xbox')) {
            let buttons = gamepad.buttons.map((button) =>{
                return button.value
              })

            let axes = gamepad.axes

            const joystickData = {
                axes: axes,
                buttons: buttons
            }
            var joystickTopic = new ROSLIB.Topic({
                ros : this.$ros,
                name : '/xbox/ra_control',
                messageType : 'sensor_msgs/Joy'
            })
            var joystickMsg = new ROSLIB.Message(joystickData)
            joystickTopic.publish(joystickMsg)
          }
        }
      }
    }, updateRate*1000)
  },

    methods: {
        updateArmEnabled: function (enabled){
            this.arm_enabled = enabled
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