<template>
  <div class="wrap">
    <div v-show="false" id="key">
      <input @keydown="keyMonitorDown" />
      <input @keyup="keyMonitorUp" />
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";

// Keycodes
const W = 87;
const A = 65;
const S = 83;
const D = 68;

// Motor power levels
const ROTATION_PWR = 1.0;
const UP_DOWN_PWR = 0.6;

export default {
  data() {
    return {
      keyboard_pub: null,

      inputData: {
        w_key: 0,
        a_key: 0,
        s_key: 0,
        d_key: 0,
      },
    };
  },

  created: function () {
    document.addEventListener("keyup", this.keyMonitorUp);
    document.addEventListener("keydown", this.keyMonitorDown);

    this.keyboard_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/mast_gimbal_cmd",
      messageType: "mrover/MastGimbal",
    });
  },

  beforeDestroy: function () {
    document.removeEventListener("keyup", this.keyMonitorUp);
    document.removeEventListener("keydown", this.keyMonitorDown);
  },

  methods: {
    // when a key is being pressed down, sets input for that key as 1
    keyMonitorDown: function (event) {
      if (event.keyCode == W) {
        this.inputData.w_key = UP_DOWN_PWR;
      } else if (event.keyCode == A) {
        this.inputData.a_key = ROTATION_PWR;
      } else if (event.keyCode == S) {
        this.inputData.s_key = UP_DOWN_PWR;
      } else if (event.keyCode == D) {
        this.inputData.d_key = ROTATION_PWR;
      }

      this.publish();
    },

    // when a key is released, sets input for that key as 0
    keyMonitorUp: function (event) {
      if (event.keyCode == W) {
        this.inputData.w_key = 0;
      } else if (event.keyCode == A) {
        this.inputData.a_key = 0;
      } else if (event.keyCode == S) {
        this.inputData.s_key = 0;
      } else if (event.keyCode == D) {
        this.inputData.d_key = 0;
      }

      this.publish();
    },

    publish: function () {
      const keyboardData = {
        left_right: this.inputData.d_key - this.inputData.a_key,
        up_down: this.inputData.w_key - this.inputData.s_key,
      };

      this.keyboard_pub.publish(keyboardData);
    },
  },
};
</script>

<style scoped>
.wrap {
  display: inline-block;
}
</style>
