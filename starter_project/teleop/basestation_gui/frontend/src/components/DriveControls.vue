<template>
  <div>
      <p>Drive Controls</p>
      <div>
        <p>Left Motor Output: {{left}}</p>
        <p>Right Motor Output:  {{right}}</p>
      </div>

  </div>
</template>

<script>

let interval;

export default {
data () {
  return {
    rotational: 0,
    linear: 0,
    left: 0,
    right: 0,

    // declaring variable
    socket: null
  }
},


beforeUnmount: function () {
  window.clearInterval(interval);
},

methods: {
  sendToROS(msg) {
    this.socket.send(JSON.stringify(msg));
  }
},

created: function () {

  const JOYSTICK_CONFIG = {
    'left_right': 0,
    'forward_back': 1,
    'twist': 2,
    'dampen': 3,
    'pan': 4,
    'tilt': 5
  }

  this.socket = new WebSocket('ws://127.0.0.1:8000/ws/drive-controls');
  this.socket.onmessage = (msg) => {
        msg = JSON.parse(msg.data)
    if(msg.type == "wheel_cmd"){
      this.left = msg.left;
      this.right = msg.right;
    }
  }

  const updateRate = 0.05;
  interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      for (let i = 0; i < 4; i++) {
        const gamepad = gamepads[i]
        if (gamepad) {
          if (gamepad.id.includes('Logitech')) {
            // -1 multiplier to make turning left a positive value
            // Both left_right axis and twisting the joystick will turn
            this.rotational = -1 * (gamepad.axes[JOYSTICK_CONFIG['left_right']] + gamepad.axes[JOYSTICK_CONFIG['twist']])
            // Range Constraints
            if (this.rotational > 1) {
              this.rotational = 1
            }
            else if (this.rotational < -1) {
              this.rotational = -1
            }
            // forward on joystick is -1, so invert
            this.linear = -1 * gamepad.axes[JOYSTICK_CONFIG['forward_back']]

            // const joystickData = {
            //   'type': 'joystick',
            //   'forward_back': this.linear,
            //   'left_right': this.rotational
            // }
        
          }
        }
        
      }

  }, updateRate*1000)
},

}
</script>

<style scoped>

</style>