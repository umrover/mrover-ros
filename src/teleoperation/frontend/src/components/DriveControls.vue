<template>
    <div class="drive">
      <!-- This component is for capturing joystick inputs -->
    </div>
  </template>
  
  <script lang="ts">
  import {inject} from "vue";
  
  let interval:number;
  
  export default {
    data() {
      return {
        joystick_pub: null,
        // websocket: inject("webSocketService") as WebSocket,
        websocket: new WebSocket('ws://localhost:8000/ws/gui'),
      };
    },
  
    beforeUnmount: function () {
      window.clearInterval(interval);
    },
  
    created: function () {
      const updateRate = 0.05;    

      interval = window.setInterval(() => {
        const gamepads = navigator.getGamepads();
        for (let i = 0; i < 4; i++) {
          const gamepad = gamepads[i];
          if (gamepad && (gamepad.id.includes("Logitech") || gamepad.id.includes("Thrustmaster"))) {
            let buttons = gamepad.buttons.map((button) => {
              return button.value;
            });
  
            const joystickData = {
              type: 'joystick_values',
              axes: gamepad.axes,
              buttons: buttons,
            };

            this.websocket.send(JSON.stringify(joystickData));
            
          }
        }
      }, updateRate * 1000);
    },
  };
  </script>
  
  <style scoped>
  .drive {
    display: none;
  }
  </style>