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
  </div>
</template>

<script>
import ROSLIB from "roslib";
import OpenLoopControl from "./OpenLoopControl.vue";

// In seconds
const updateRate = 0.1;

let interval;

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

  beforeDestroy: function () {
    window.clearInterval(interval);
  },

  created: function () {
    this.cache_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "cache_cmd",
      messageType: "sensor_msgs/JointState",
    });

    interval = window.setInterval(() => {
      this.cache_pub.publish(this.messageObject);
    }, updateRate * 1000);
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
