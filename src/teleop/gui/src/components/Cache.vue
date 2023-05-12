<template>
  <div>
    <h3>Cache Controls</h3>
    <p>Close cache with 'n' and open with 'm'.</p>
    <OpenLoopControl
      :forwards-key="77"
      :backwards-key="78"
      @velocity="velocity = $event"
    ></OpenLoopControl>
    <div class="limit-switch">
      <LEDIndicator
        :connected="limit_switch_pressed"
        :name="'Cache Limit Switch'"
        :show_name="true"
      />
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import OpenLoopControl from "./OpenLoopControl.vue";
import LEDIndicator from "./LEDIndicator.vue";

// In seconds
const updateRate = 0.1;

let interval;

export default {
  components: {
    OpenLoopControl,
    LEDIndicator
  },

  data() {
    return {
      limit_switch_pressed: false,
      velocity: 0,

      cache_pub: null,
      limit_switch_sub: null
    };
  },

  computed: {
    messageObject: function () {
      const msg = {
        name: ["cache"],
        position: [],
        velocity: [this.velocity],
        effort: []
      };
      return new ROSLIB.Message(msg);
    }
  },

  beforeDestroy: function () {
    window.clearInterval(interval);
  },

  created: function () {
    this.cache_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "cache_cmd",
      messageType: "sensor_msgs/JointState"
    });

    this.limit_switch_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "cache_limit_switch_data",
      messageType: "mrover/LimitSwitchData"
    });

    this.limit_switch_sub.subscribe((msg) => {
      this.limit_switch_pressed = msg.limit_a_pressed;
    });

    interval = window.setInterval(() => {
      this.cache_pub.publish(this.messageObject);
    }, updateRate * 1000);
  }
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

.limit-switch {
  margin-top: 10px;
}
</style>
