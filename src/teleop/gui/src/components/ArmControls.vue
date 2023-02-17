<template>
  <div class="wrap">
    <h3>Arm controls</h3>
    <div class="controls">
      <!-- Make opposite option disappear so that we cannot select both -->
      <!-- Change to radio buttons in the future -->
      <input type="radio" ref="arm-enabled" v-model="arm_controls" :name="'Arm Enabled'" value="arm_disabled" @change="updateArmEnabled()"> Arm Disabled
      <input type="radio" ref="open-loop-enabled" v-model="arm_controls" :name="'Open Loop Enabled'" value="open_loop" @change="updateArmEnabled()"> Open Loop
      <input type="radio" ref="servo-enabled" v-model="arm_controls" :name="'Servo'" value="servo" @change="updateArmEnabled()"> Servo
    </div>
    <h3>Joint Locks</h3>
    <div class="controls">
      <Checkbox ref="A" :name="'A'" @toggle="updateJointsEnabled(0, $event)" />
      <Checkbox ref="B" :name="'B'" @toggle="updateJointsEnabled(1, $event)" />
      <Checkbox ref="C" :name="'C'" @toggle="updateJointsEnabled(2, $event)" />
      <Checkbox ref="D" :name="'D'" @toggle="updateJointsEnabled(3, $event)" />
      <Checkbox ref="E" :name="'E'" @toggle="updateJointsEnabled(4, $event)" />
      <Checkbox ref="F" :name="'F'" @toggle="updateJointsEnabled(5, $event)" />
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import Checkbox from "./Checkbox.vue";

// In seconds
const updateRate = 0.1;
let interval;

export default {
  components: {
    Checkbox
  },
  data() {
    return {
      armcontrols_pub: null,
      arm_controls: "arm_disabled",
      joystick_pub: null,
      joystickservo_pub: null,
      jointlock_pub: null,
      jointstate_pub: null,
      joints_array: [false, false, false, false, false, false]
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
    this.updateArmEnabled();
    this.joystick_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/xbox/ra_open_loop",
      messageType: "sensor_msgs/Joy"
    });
    this.joystickservo_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/xbox/ra_servo",
      messageType: "sensor_msgs/Joy"
    });
    this.jointlock_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/joint_lock",
      messageType: "mrover/JointLock"
    });
    this.jointstate_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/ra_cmd",
      messageType: "sensors_msgs/JointState"
    });
    const jointData = {
      //publishes array of all falses when refreshing the page
      joints: this.joints_array
    };
    var jointlockMsg = new ROSLIB.Message(jointData);
    this.jointlock_pub.publish(jointlockMsg);

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
              this.publishJoystickMessage(gamepad.axes, buttons);
            }
          }
        }
      }
    }, updateRate * 1000);
  },

  methods: {
    updateArmEnabled: function () {
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
    publishJoystickMessage: function (axes, buttons) {
      const joystickData = {
        axes: axes,
        buttons: buttons
      };
      var joystickMsg = new ROSLIB.Message(joystickData);
      if (this.arm_enabled) {
        if (this.servo_enabled) {
          this.joystickservo_pub.publish(joystickMsg);
        } else {
          this.joystick_pub.publish(joystickMsg);
        }
      }
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
