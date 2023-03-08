<template>
  <div class="wrap">
    <h3>Arm controls</h3>
    <div class="controls">
      <Checkbox
        ref="arm-enabled"
        :name="'Arm Enabled'"
        @toggle="updateArmEnabled($event)"
      />
    </div>
    <div class="controls laser">
      <ToggleButton
        :current-state="laser_enabled"
        label-enable-text="Arm Laser On"
        label-disable-text="Arm Laser Off"
        @change="toggleArmLaser()"
      />
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import Checkbox from "./Checkbox.vue";
import ToggleButton from "./ToggleButton.vue";

// In seconds
const updateRate = 0.1;
let interval;

export default {
  components: {
    Checkbox,
    ToggleButton
  },
  data() {
    return {
      arm_enabled: false,
      laser_enabled: false,
      joystick_pub: null,
      laser_service: null
    };
  },

  beforeDestroy: function () {
    window.clearInterval(interval);
  },

  created: function () {
    this.joystick_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/xbox/ra_control",
      messageType: "sensor_msgs/Joy"
    });
    this.laser_service = new ROSLIB.Service({
      ros: this.$ros,
      name: "change_arm_laser_state",
      serviceType: "mrover/ChangeDeviceState"
    });
    interval = window.setInterval(() => {
      if (this.arm_enabled) {
        const gamepads = navigator.getGamepads();
        for (let i = 0; i < 4; i++) {
          const gamepad = gamepads[i];
          if (gamepad) {
            if (
              gamepad.id.includes("Microsoft") ||
              gamepad.id.includes("Xbox")
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
      }
    }, updateRate * 1000);
  },

  methods: {
    updateArmEnabled: function (enabled) {
      this.arm_enabled = enabled;
    },
    toggleArmLaser: function () {
      this.laser_enabled = !this.laser_enabled;
      let request = new ROSLIB.ServiceRequest({
        enable: this.laser_enabled
      });
      this.laser_service.callService(request, (result) => {
        if (!result) {
          this.laser_enabled = !this.laser_enabled;
          alert("Toggling Arm Laser failed.");
        }
      });
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

.laser {
  padding-top: 10px;
}
</style>
