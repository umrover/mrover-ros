<template>
  <div>
    <h3>Cache Controls</h3>
    <!-- Left and right arrows keys -->
    <p>Control with left and right arrows keys</p>
    <OpenLoopControl
      :forwards-key="39"
      :backwards-key="37"
      @velocity="velocity = $event"
    ></OpenLoopControl>

    <!-- Cache is open loop for now 11/9/22 -->
    <!-- TODO: Add Indicator for cache being opened or closed once we have closed loop -->
  </div>
</template>

<script>
import ROSLIB from "roslib";
import OpenLoopControl from "./OpenLoopControl.vue";

export default {
  components: {
    OpenLoopControl,
  },

  data() {
    return {
      velocity: 0,

      cache_pub: null,
    };
  },

  computed: {
    messageObject: function () {
      const msg = {
        name: ["cache"],
        position: [],
        velocity: [this.velocity],
        effort: [],
      };
      return new ROSLIB.Message(msg);
    },
  },

  watch: {
    messageObject: function (msg) {
      this.cache_pub.publish(msg);
    },
  },
  created: function () {
    this.cache_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "cache_cmd",
      messageType: "sensor_msgs/JointState",
    });
    this.cache_pub.publish(this.messageObject);
  },
};
</script>

<style scoped>
#circle {
  height: 20px;
  width: 20px;
  border-radius: 50%;
  background-color: green;
  margin-right: 2%;
}

#box-1 {
  margin-top: 5%;
  display: flex;
  flex-direction: row;
}
</style>
