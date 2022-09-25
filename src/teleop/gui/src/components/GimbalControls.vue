<template>
    <div class="wrap">
        <div v-show='false' id="key">
          <input v-on:keydown="keymonitordown">
          <input v-on:keyup="keymonitorup">
        </div>
        <p>Gimbal Controls</p>
    </div>
</template>

<script>

import ROSLIB from "roslib"

export default {

    data() {
    return {

        keyboard_pub: null,

        inputData : {
            w_key: 0,
            a_key: 0,
            s_key: 0,
            d_key: 0
        } 
    }
    },

    created: function () {
        document.addEventListener('keydown', this.keymonitordown);
        document.addEventListener('keyup', this.keymonitorup);

        this.keyboard_pub = new ROSLIB.Topic({
            ros : this.$ros,
            name : '/gimbal_control',
            messageType : 'mrover/GimbalCmd'
        });
    },

  methods: {
    keymonitordown: function(event) {

        if (event.keyCode == 87) {
            this.inputData.w_key = 1;
        }
        else if (event.keyCode == 65) {
            this.inputData.a_key = 1;
        }
        else if (event.keyCode == 83) {
            this.inputData.s_key = 1;
        }
        else if (event.keyCode == 68) {
            this.inputData.d_key = 1;
        }

        this.publish()
    },

    keymonitorup: function(event) {
        if (event.keyCode == 87) {
            this.inputData.w_key = 0;
        }
        else if (event.keyCode == 65) {
            this.inputData.a_key = 0;
        }
        else if (event.keyCode == 83) {
            this.inputData.s_key = 0;
        }
        else if (event.keyCode == 68) {
            this.inputData.d_key = 0;
        }

        this.publish()
    },

    publish: function() {
        const keyboardData = {
            "left_right": this.inputData.d_key - this.inputData.a_key,
            "up_down": this.inputData.w_key - this.inputData.s_key
        };

        this.keyboard_pub.publish(keyboardData)
    }

  },

  beforeDestroy: function () {
    document.removeEventListener('keydown', this.keymonitordown);
    document.removeEventListener('keyup', this.keymonitorup);
  }
}
</script>

<style scoped>
  .wrap {
    display: inline-block;
  }
</style>