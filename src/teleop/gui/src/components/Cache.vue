<template>
  <div>
    <h3>Cache Controls</h3>
      <div class="box1">
        <ToggleButton
          id="cache_enabled"
          :current-state="enabled"
          label-enable-text="Open Loop"
          label-disable-text="Disabled"
          @change="toggleEnabled"
        />
        <div class="controls">
          <div v-if="enabled">
            <p>Close cache with C and open with O</p>
            <OpenLoopControl
              :forwards-key="79"
              :backwards-key="67"
              :scale-default="100"
              @velocity="velocity = $event"
            ></OpenLoopControl>
          </div>
        </div>
        <div class="cache_limit_switch">
          <LEDIndicator
            :connected="limit_switch_pressed"
            :name="'Cache Limit Switch'"
            :show_name="true"
          />
        </div>
      </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import OpenLoopControl from "./OpenLoopControl.vue";
import LEDIndicator from "./LEDIndicator.vue";
import ToggleButton from "./ToggleButton.vue";

// In seconds
const updateRate = 0.1;

let interval;

export default {
  components: {
    OpenLoopControl,
    LEDIndicator,
    ToggleButton,
  },

  data() {
    return {
      enabled: false,

      limit_switch_pressed: false,
      velocity: 0,

      cache_pub: null,
      limit_switch_sub: null
    };
  },

  computed: {
    messageObject: function () {
      if (this.enabled) {
        const msg = {
          name: ["cache"],
          position: [],
          velocity: [this.velocity],
          effort: []
        };
        return new ROSLIB.Message(msg);
      }
      else {
        const msg = {
          name: ["cache"],
          position: [],
          velocity: [0],
          effort: []
        };
        return new ROSLIB.Message(msg);
      }
    }
  },

  beforeUnmount: function () {
    this.enabled = false;
    this.cache_pub.publish(this.messageObject);

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
  },

  methods: {
    toggleEnabled: function () {
      this.enabled = !this.enabled;
    }
  },
};
</script>

<style scoped>
.controls {
  display: inline-block;
  margin-top: 10px;
}

.box1 {
  display: flex;
  flex-direction: column;
  justify-content: space-between;
}

.cache_limit_switch {
  margin-top: 10px;
  margin-left: -22px;
}
</style>
