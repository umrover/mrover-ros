<template>
  <div class="drive">
    <!-- This component is for capturing joystick inputs -->
  </div>
</template>

<script>
import ROSLIB from "roslib";

let interval;

export default {
  data() {
    return {
      joystick_pub: null,
    };
  },

  beforeUnmount: function () {
    window.clearInterval(interval);
  },

  created: function () {
    const updateRate = 0.05;
    this.joystick_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/joystick",
      messageType: "sensor_msgs/Joy",
    });
    interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads();
      for (let i = 0; i < 4; i++) {
        const gamepad = gamepads[i];
        if (gamepad && (gamepad.id.includes("Logitech") || gamepad.id.includes("Thrustmaster"))) {
          let buttons = gamepad.buttons.map((button) => {
            return button.value;
          });

          const joystickData = {
            axes: gamepad.axes,
            buttons: buttons,
          };

          var joystickMsg = new ROSLIB.Message(joystickData);
          this.joystick_pub.publish(joystickMsg);
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
