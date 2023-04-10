<template>
  <div class="wrap">
    <h2>Arm Controls</h2>
    <div class="controls-flex">
      <h4>Arm mode</h4>
      <!-- Make opposite option disappear so that we cannot select both -->
      <!-- Change to radio buttons in the future -->
      <input
        ref="arm-enabled"
        v-model="arm_controls"
        type="radio"
        :name="'Arm Enabled'"
        value="arm_disabled"
        @change="updateArmMode()"
      />
      Arm Disabled
      <input
        ref="open-loop-enabled"
        v-model="arm_controls"
        type="radio"
        :name="'Open Loop Enabled'"
        value="open_loop"
        @change="updateArmMode()"
      />
      Open Loop
      <input
        ref="servo-enabled"
        v-model="arm_controls"
        type="radio"
        :name="'Servo'"
        value="servo"
        @change="updateArmMode()"
      />
      Servo
    </div>
    <div class="controls-flex">
      <h4>Joint Locks</h4>
      <Checkbox ref="A" :name="'A'" @toggle="updateJointsEnabled(0, $event)" />
      <Checkbox ref="B" :name="'B'" @toggle="updateJointsEnabled(1, $event)" />
      <Checkbox ref="C" :name="'C'" @toggle="updateJointsEnabled(2, $event)" />
      <Checkbox ref="D" :name="'D'" @toggle="updateJointsEnabled(3, $event)" />
      <Checkbox ref="E" :name="'E'" @toggle="updateJointsEnabled(4, $event)" />
      <Checkbox ref="F" :name="'F'" @toggle="updateJointsEnabled(5, $event)" />
    </div>
    <div class="controls-flex">
      <h4>Misc. Controls</h4>
      <Checkbox
      ref="Slow Mode"
      :name="'Slow Mode'"
      @toggle="updateSlowMode($event)"
      />
      <ToggleButton
      :current-state="laser_enabled"
      label-enable-text="Arm Laser On"
      label-disable-text="Arm Laser Off"
      @change="toggleArmLaser()"
      />
    </div>
    <div class="controls-flex">
      <h4>Calibration</h4>
      <CalibrationCheckbox
        name="Joint B Calibration"
        joint_name="joint_b"
        calibrate_topic="ra_is_calibrated"
      />
      <JointAdjust />
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import Checkbox from "./Checkbox.vue";
import ToggleButton from "./ToggleButton.vue";
import CalibrationCheckbox from "./CalibrationCheckbox.vue";
import JointAdjust from "./JointAdjust.vue";

// In seconds
const updateRate = 0.1;
let interval;

export default {
  components: {
    CalibrationCheckbox,
    Checkbox,
    JointAdjust,
    ToggleButton
  },
  data() {
    return {
      armcontrols_pub: null,
      arm_controls: "arm_disabled",
      joystick_pub: null,
      jointlock_pub: null,
      joints_array: [false, false, false, false, false, false],
      slow_mode: false,
      slowmode_pub: null,
      laser_enabled: false,
      laser_service: null
    };
  },

  beforeDestroy: function () {
    window.clearInterval(interval);
  },

  created: function () {
    this.armcontrols_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "ra/mode",
      messageType: "std_msgs/String"
    });
    this.updateArmMode();
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
    this.jointlock_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/joint_lock",
      messageType: "mrover/JointLock"
    });
    this.slow_mode_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/ra_slow_mode",
      messageType: "std_msgs/Bool"
    });
    const jointData = {
      //publishes array of all falses when refreshing the page
      joints: this.joints_array
    };
    var jointlockMsg = new ROSLIB.Message(jointData);
    this.jointlock_pub.publish(jointlockMsg);

    interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads();
      for (let i = 0; i < 4; i++) {
        const gamepad = gamepads[i];
        if (gamepad) {
          // Microsoft and Xbox for old Xbox 360 controllers
          // X-Box for new PowerA Xbox One controllers
          if (
            gamepad.id.includes("Microsoft") ||
            gamepad.id.includes("Xbox") ||
            gamepad.id.includes("X-Box")
          ) {
            let buttons = gamepad.buttons.map((button) => {
              return button.value;
            });
            this.publishJoystickMessage(gamepad.axes, buttons);
          }
        }
      }
    }, updateRate * 1000);
  },

  methods: {
    updateArmMode: function () {
      const armData = {
        data: this.arm_controls
      };
      var armcontrolsmsg = new ROSLIB.Message(armData);
      this.armcontrols_pub.publish(armcontrolsmsg);
    },

    updateJointsEnabled: function (jointnum, enabled) {
      this.joints_array[jointnum] = enabled;
      const jointData = {
        joints: this.joints_array
      };
      var jointlockMsg = new ROSLIB.Message(jointData);
      this.jointlock_pub.publish(jointlockMsg);
    },

    updateSlowMode: function (enabled) {
      this.slow_mode = enabled;
      const slowData = {
        data: this.slow_mode
      };
      var slowModeMsg = new ROSLIB.Message(slowData);
      this.slow_mode_pub.publish(slowModeMsg);
    },
    publishJoystickMessage: function (axes, buttons) {
      const joystickData = {
        axes: axes,
        buttons: buttons
      };
      var joystickMsg = new ROSLIB.Message(joystickData);
      this.joystick_pub.publish(joystickMsg);
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
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-items: center;
  width: 100%;
}

.wrap h2 {
  margin: 0;
  padding: 0;
  font-size: 1.5em;
  font-weight: bold;
  text-align: center;
  width: 100%;
  padding-top: 5px;
  padding-bottom: 5px;
}

.controls-flex {
  display: flex;
  align-items: center;
  width: 100%;
  column-gap: 10px;
  border: 1px solid black;
  padding-left: 10px;
  margin-bottom: 5px;
  margin-top: 5px;
  width: calc(100% - 10px);
  background-color: rgb(180, 180, 180);
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
