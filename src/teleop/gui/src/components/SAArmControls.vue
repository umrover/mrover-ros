<template>
  <div class="wrap">
    <h3>SA Arm controls</h3>
    <div class="controls">
      <input
        ref="arm-disabled"
        v-model="arm_mode"
        type="radio"
        :name="'SA Arm Disabled'"
        value="sa_disabled"
      />
      SA Disabled
      <input
        ref="open-loop-enabled"
        v-model="arm_mode"
        type="radio"
        :name="'Open Loop Enabled'"
        value="open_loop"
      />
      Open Loop
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";

let interval;
// In seconds
const updateRate = 0.1;

export default {
  data() {
    return {
      arm_mode: "sa_disabled",
      joystick_pub: null,
      sa_mode_service: null
    };
  },

  watch: {
    arm_mode: function (newMode, oldMode) {
      this.updateArmMode(newMode, oldMode);
    }
  },

  beforeUnmount: function () {
    this.updateArmMode("sa_disabled", this.arm_mode);
    window.clearInterval(interval);
  },

  created: function () {
    this.joystick_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/xbox/sa_control",
      messageType: "sensor_msgs/Joy"
    });
    this.sa_mode_service = new ROSLIB.Service({
      ros: this.$ros,
      name: "change_sa_mode",
      serviceType: "mrover/ChangeArmMode"
    });
    this.updateArmMode("sa_disabled", this.arm_mode);
    interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads();
      for (let i = 0; i < 4; i++) {
        const gamepad = gamepads[i];
        if (gamepad) {
          if (
            gamepad.id.includes("Microsoft") ||
            gamepad.id.includes("Xbox") ||
            gamepad.id.includes("X-Box")
          ) {
            let buttons = gamepad.buttons.map((button) => {
              return button.value;
            });

            const joystickData = {
              axes: gamepad.axes,
              buttons: buttons
            };
            var joystickMsg = new ROSLIB.Message(joystickData);
            this.joystick_pub.publish(joystickMsg);
          }
        }
      }
    }, updateRate * 1000);
  },

  methods: {
    updateArmMode: function (newMode, oldMode) {
      this.sa_mode_service.callService(
        new ROSLIB.ServiceRequest({ mode: newMode }),
        (result) => {
          if (!result.success) {
            alert("Failed to enable SA arm");
            this.arm_mode = oldMode;
          }
        }
      );
    }
  }
};
</script>

<style scoped>
.wrap {
  display: inline-block;
  align-items: center;
  justify-items: center;
  width: 100%;
}
.controls {
  display: flex;
  align-items: center;
}
.header {
  display: flex;
  align-items: center;
}
.joint-b-calibration {
  display: flex;
  gap: 10px;
  width: 250px;
  font-weight: bold;
  color: red;
}
</style>
