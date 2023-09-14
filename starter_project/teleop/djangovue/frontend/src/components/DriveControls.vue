<template>
  <div>
      <p>Drive Controls</p>
      <p>{{ forward }}</p>
      <p> {{ left }}</p>
  </div>
</template>

<script>

import axios from 'axios';

let interval;

export default {
data () {
  return {
    rotational: 0,
    linear: 0,

    left: 0,
    forward: 0
  }
},


beforeUnmount: function () {
  window.clearInterval(interval);
},

methods: {
  getVals : function() {
    axios.get('/api/joystick/').then(
      response => {
        this.forward = response.data[0].forward_back;
        this.left = response.data[0].left_right;
      }
    );
  },
  postVals: function() {
    axios.post('/api/joystick/', 
    {
      forward_back: 123.456, 
      left_right: 987.654
    }).then(
      response => {
        this.left = response.data;
      }
    );
  },

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

  this.postVals();

  const updateRate = 1;
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
            //   'forward_back': this.linear,
            //   'left_right': this.rotational,
            // }
          }
        }
        
      }

      this.getVals();
  }, updateRate*1000)
},

}
</script>

<style scoped>

</style>