<template>
    <div class="drive">
      <!-- This component is for capturing joystick inputs -->
    </div>
  </template>
  
  <script>
  
  let interval;
  
  export default {
    data() {
      return {
        joystick_pub: null,
        socket: null,
      };
    },
  
    beforeDestroy: function () {
      window.clearInterval(interval);
    },
  
    created: function () {
      const updateRate = 0.05;

      this.socket = new WebSocket('ws://127.0.0.1:8000/ws/drive-controls');
    

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

            this.socket.send(JSON.stringify(joystickData));
            
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