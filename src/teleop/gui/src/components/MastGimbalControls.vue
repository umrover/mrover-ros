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

const UPDATE_RATE_S = 1;
let interval;

export default {
  data() {
    return {
      rotation_pwr: 0,
      up_down_pwr: 0,

      keyboard_pub: null,

      inputData: {
        w_key: 0,
        a_key: 0,
        s_key: 0,
        d_key: 0
      }
    };
  },

  beforeUnmount: function () {
    window.clearInterval(interval);
    document.removeEventListener("keyup", this.keyMonitorUp);
    document.removeEventListener("keydown", this.keyMonitorDown);
  },

  created: function () {
    // get power levels for gimbal controls.
    let config = new ROSLIB.Param({
      ros: this.$ros,
      name: "teleop/mast_gimbal_power"
    });
    config.get((value) => {
      this.rotation_pwr = value.rotation_pwr;
      this.up_down_pwr = value.up_down_pwr;
    });

    // Add key listeners.
    document.addEventListener("keyup", this.keyMonitorUp);
    document.addEventListener("keydown", this.keyMonitorDown);

    this.keyboard_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/mast_gimbal_cmd",
      messageType: "mrover/MastGimbal"
    });

    // Publish periodically in case a topic message is missed.
    interval = window.setInterval(() => {
      this.publish();
    }, UPDATE_RATE_S * 1000);
  },

  methods: {
    // When a key is being pressed down, set the power level.
    // Ignore keys that are already pressed to avoid spamming when holding values.
    keyMonitorDown: function (event) {
      if (event.key.toLowerCase() == "w") {
        if (this.inputData.w_key > 0) {
          return;
        }
        this.inputData.w_key = this.up_down_pwr;
      } else if (event.key.toLowerCase() == "a") {
        if (this.inputData.a_key > 0) {
          return;
        }
        this.inputData.a_key = this.rotation_pwr;
      } else if (event.key.toLowerCase() == "s") {
        if (this.inputData.s_key > 0) {
          return;
        }
        this.inputData.s_key = this.up_down_pwr;
      } else if (event.key.toLowerCase() == "d") {
        if (this.inputData.d_key > 0) {
          return;
        }
        this.inputData.d_key = this.rotation_pwr;
      }

      this.publish();
    },

    // when a key is released, sets input for that key as 0
    keyMonitorUp: function (event) {
      if (event.key.toLowerCase() == "w") {
        this.inputData.w_key = 0;
      } else if (event.key.toLowerCase() == "a") {
        this.inputData.a_key = 0;
      } else if (event.key.toLowerCase() == "s") {
        this.inputData.s_key = 0;
      } else if (event.key.toLowerCase() == "d") {
        this.inputData.d_key = 0;
      }

      this.publish();
    },

    publish: function () {
      let keyboardData;

      if (this.textSelected()) {
        keyboardData = {
          left_right: 0,
          up_down: 0
        };
      } else {
        keyboardData = {
          left_right: this.inputData.d_key - this.inputData.a_key,
          up_down: this.inputData.w_key - this.inputData.s_key
        };
      }

      this.keyboard_pub.publish(keyboardData);
    },

    textSelected: function () {
      let active = document.activeElement;
      return active.tagName.toLowerCase() == "input" && active.type == "text";
    }
  }
};
</script>

<style scoped>
.wrap {
  display: inline-block;
}
</style>
