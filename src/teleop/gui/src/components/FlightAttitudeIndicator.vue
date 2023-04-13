<template>
  <div>
    <div><h3>Flight Attitude Indicator</h3></div>
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
      pitch: 0,
      roll: 0,
      euler:[]
    };
  },

  created: function () {
    this.tfClient = new ROSLIB.TFClient({
      ros: this.$ros,
      fixedFrame: "map",
      // Thresholds to trigger subscription callback
      angularThres: 0.01,
      transThres: 0.01
    });

    // Subscriber for odom to base_link transform
    this.tfClient.subscribe("base_link", (tf) => {
      // Callback for IMU quaternion that describes bearing
      this.euler = qte(tf.rotation)
      this.pitch = this.euler[1]
      this.roll=this.euler[0]
    });
  }
};
</script>

<style scoped></style>
