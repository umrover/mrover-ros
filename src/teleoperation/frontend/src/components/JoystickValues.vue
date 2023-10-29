<template>
    <div class="datagrid">
      <div>
        <p style="margin-top: 0px">
          left_right: {{joystick_values.left_right.toFixed(3) }}
        </p>
        <p>forward_back: {{ joystick_values.forward_back.toFixed(3) }}</p>
        <p>twist: {{ joystick_values.twist.toFixed(3) }}</p>
      </div>
      <div>
        <p style="margin-top: 0px">
          dampen: {{ joystick_values.dampen.toFixed(3) }}
        </p>
        <p>pan: {{ joystick_values.pan.toFixed(3) }}</p>
        <p>tilt: {{ joystick_values.tilt.toFixed(3) }}</p>
      </div>
    </div>
  </template>
  
  <script lang="ts">

import {inject} from "vue"

  interface JoystickValues {
    left_right: number,
    forward_back: number,
    twist: number,
    dampen: number,
    pan: number,
    tilt: number,
  }
  
  export default {
    data() {
      return {
        websocket: inject("webSocketService") as WebSocket,
        joystick_mappings: {},
        joystick_values: {
          left_right: 0,
          forward_back: 0,
          twist: 0,
          dampen: 0,
          pan: 0,
          tilt: 0,
        } as JoystickValues,
      };
    },
  
    created: function () {
        this.websocket.onmessage = (msg) => {
          console.log("here")
          if (msg.type == "joystick") {
            this.joystick_values.left_right = msg.left_right;
            this.joystick_values.forward_back = msg.forward_back;
            this.joystick_values.twist = msg.twist;
            this.joystick_values.dampen = msg.dampen;
            this.joystick_values.pan = msg.pan;
            this.joystick_values.tilt = msg.tilt;
          }
        }
      // get joystick mappings
    //   let a = new ROSLIB.Param({
    //     ros: this.$ros,
    //     name: "teleoperations/joystick_mappings",
    //   });
    //   a.get((value) => {
    //     this.joystick_mappings = value;
    //   });

    },
  };
  </script>
  <style scoped>
  .datagrid {
    display: grid;
    grid-template-columns: 50% 50%;
    line-height: 0.5em;
  }
  </style>