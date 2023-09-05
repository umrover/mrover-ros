<template>
  <div>
    <Attitude :size="200" :pitch="pitch" :roll="roll"></Attitude>
  </div>
</template>

<script>
import { Attitude } from "vue-flight-indicators";
import ROSLIB from "roslib";
import * as qte from "quaternion-to-euler";

export default {
  components: {
    Attitude
  },
  data() {
    return {
      // Pitch and Roll in Degrees
      pitch: 0,
      roll: 0,
      euler: []
    };
  },

  created: function () {
    this.tfClient = new ROSLIB.TFClient({
      ros: this.$ros,
      fixedFrame: "map",
      // Thresholds to trigger subscription callback
      angularThres: 0.0001,
      transThres: 0.0001
    });

    // Subscriber for odom to base_link transform
    this.tfClient.subscribe("base_link", (tf) => {
      // Callback for IMU quaternion that describes bearing
      let quat = [tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z];
      this.euler = qte(quat);
      this.pitch = this.euler[0] * 180/Math.PI;
      this.roll = this.euler[1] * 180/Math.PI;
    });
  }
};
</script>

<style scoped></style>
