<template>
  <div>
    Velocity Commands<br />
    Linear x: {{ linear_x }} m/s<br />
    Angular z: {{ angular_z }} rad/s
  </div>
</template>

<script>
import ROSLIB from "roslib/src/RosLib";

export default {
  data() {
    return {
      linear_x: 0,
      angular_z: 0,

      //Pubs and Subs
      cmd_vel_sub: null,
    };
  },

  created: function () {
    (this.cmd_vel_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "cmd_vel",
      messageType: "geometry_msgs/Twist",
    })),
      this.cmd_vel_sub.subscribe((msg) => {
        // Only use linear x and angular z
        this.linear_x = msg.linear.x;
        this.angular_z = msg.angular.z;
      });
  },
};
</script>

<style scoped></style>
