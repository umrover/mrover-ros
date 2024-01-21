<template>
<<<<<<< HEAD
    <div class="drive">
      <!-- This component is for capturing joystick inputs -->
    </div>
  </template>
  
  <script lang="ts">
  import {inject} from "vue";
  import { mapActions } from 'vuex';
  
  let interval:number;
  
  export default {
    data() {
      return {
        // websocket: inject("webSocketService") as WebSocket,
        // websocket: new WebSocket('ws://localhost:8000/ws/gui'),
      };
    },

    methods: {
      ...mapActions('websocket', ['sendMessage'])
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

            this.sendMessage(joystickData);
            
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
=======
  <div class="drive">
    <!-- This component is for capturing joystick inputs -->
  </div>
</template>

<script lang="ts">
import { mapActions } from 'vuex'

let interval: number

export default {
  data() {
    return {}
  },

  methods: {
    ...mapActions('websocket', ['sendMessage'])
  },

  beforeUnmount: function () {
    window.clearInterval(interval)
  },

  created: function () {
    const updateRate = 0.05

    interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      for (let i = 0; i < 4; i++) {
        const gamepad = gamepads[i]
        if (gamepad && (gamepad.id.includes('Logitech') || gamepad.id.includes('Thrustmaster'))) {
          let buttons = gamepad.buttons.map((button) => {
            return button.value
          })

          const joystickData = {
            type: 'joystick_values',
            axes: gamepad.axes,
            buttons: buttons
          }

          this.sendMessage(joystickData)
        }
      }
    }, updateRate * 1000)
  }
}
</script>

<style scoped>
.drive {
  display: none;
}
</style>
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
