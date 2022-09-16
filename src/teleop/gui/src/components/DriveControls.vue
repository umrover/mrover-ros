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
      dampen: 0,
      reverse: false,
      drive_config: null,
      joystick_mapping: null
    }
  },


  beforeDestroy: function () {
    window.clearInterval(interval);
  },

  created: function () {

    var drive_control_param = new ROSLIB.Param({
      ros : this.$ros,
      name :  '/teleop/drive_controls'
    });

    
    var joystick_mapping_param = new ROSLIB.Param({
      ros : this.$ros,
      name :  '/teleop/joystick_mappings'
    });

    drive_control_param.get((value) => {
      if (value != null) {
          this.drive_config = value;
          console.log(this.drive_config.forward_back.multiplier)
      }
    });
      
    joystick_mapping_param.get((value) => {
      if (value != null) {
        this.joystick_mapping = value;
        console.log(this.joystick_mapping)
      }
    });


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

              //Deadzone applied to all axis
              const deadzone = 0.05
              let axes = gamepad.axes.map((axis) => {
                return Math.abs(axis) <= deadzone ? 0 : axis
              })
              
              //Invert Forward/Back Stick, forward is normally -1
              axes[this.joystick_mapping.forward_back] = this.drive_config.forward_back.multiplier * axes[this.joystick_mapping.forward_back]
              axes[this.joystick_mapping.left_right] = this.drive_config.left_right.multiplier * axes[this.joystick_mapping.left_right]

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