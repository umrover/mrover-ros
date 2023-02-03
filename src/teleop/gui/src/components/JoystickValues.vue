<template>
  <div class="datagrid">
    <div>
      <p style="margin-top: 0px">
        left_right: {{ joystick_values.left_right }}
      </p>
      <p>forward_back: {{ joystick_values.forward_back }}</p>
      <p>twist: {{ joystick_values.twist }}</p>
    </div>
    <div>
      <p style="margin-top: 0px">dampen: {{ joystick_values.dampen }}</p>
      <p>pan: {{ joystick_values.pan }}</p>
      <p>tilt: {{ joystick_values.tilt }}</p>
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib/src/RosLib";

export default {
  data() {
    return {
      joystick_mappings: {},
      joystick_values: {
        left_right: 0,
        forward_back: 0,
        twist: 0,
        dampen: 0,
        pan: 0,
        tilt: 0,
      },

      // Pubs and Subs
      joystick_sub: null,
    };
  },

  created: function () {
    // get joystick mappings
    let a = new ROSLIB.Param({
      ros: this.$ros,
      name: "teleop/joystick_mappings",
    });
    a.get((value) => {
      this.joystick_mappings = value;
    });

    this.joystick_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "joystick",
      messageType: "sensor_msgs/Joy",
    });

    this.joystick_sub.subscribe((msg) => {
      // Callback for joystick values
      this.joystick_values.left_right =
        msg.axes[this.joystick_mappings.left_right];
      this.joystick_values.forward_back =
        msg.axes[this.joystick_mappings.forward_back];
      this.joystick_values.twist = msg.axes[this.joystick_mappings.twist];
      this.joystick_values.dampen = msg.axes[this.joystick_mappings.dampen];
      this.joystick_values.pan = msg.axes[this.joystick_mappings.pan];
      this.joystick_values.tilt = msg.axes[this.joystick_mappings.tilt];
    });
  },
};
</script>
<style scoped>
.datagrid {
  display: grid;
  grid-template-columns: 50% 50%;
  line-height: 0.5em;
}
</style>
