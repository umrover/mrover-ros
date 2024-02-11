<template>
  <div>
    <h3>Carousel Controls</h3>
    <div class="box1">
      <ToggleButton
        id="carousel_open_loop"
        :current-state="openLoop"
        label-enable-text="Open Loop"
        label-disable-text="Disabled"
        @change="toggleOpenLoop"
      />
      <div class="controls">
        <div v-if="openLoop">
          <p>Control with X (Forward) and Z (Backwards)</p>
          <OpenLoopControl
            :forwards-key="88"
            :backwards-key="90"
            :scale-default="50"
            @velocity="velocity = $event"
          />
        </div>
        <!-- <div v-else>
          <label for="position">Rotate carousel to position: </label>
          <input v-model="site" type="radio" value="A" />A
          <input v-model="site" type="radio" value="B" />B
          <input v-model="site" type="radio" value="C" />C
          <CalibrationCheckbox
            :name="'Carousel Calibration'"
            :joint_name="'carousel'"
            :calibrate_topic="'carousel_is_calibrated'"
          />
          <MotorAdjust :options="[{ name: 'carousel', option: 'Carousel' }]" />
          <LimitSwitch
            :switch_name="'carousel'"
            :name="'Carousel Limit Switch'"
          />
        </div> -->
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import ToggleButton from "./ToggleButton.vue";
import OpenLoopControl from "./OpenLoopControl.vue";
import CalibrationCheckbox from "./CalibrationCheckbox.vue";
import MotorAdjust from "./MotorAdjust.vue";
import LimitSwitch from "./LimitSwitch.vue";

// In seconds
const updateRate = 0.1;

let interval;

export default {
  components: {
    OpenLoopControl,
    CalibrationCheckbox,
    MotorAdjust,
    LimitSwitch,
    ToggleButton,
  },
  data() {
    return {
      openLoop: false,
      velocity: 0,
      site: "A",

      carousel_pub: null,
    };
  },

  computed: {
    messageObject: function () {
      if (this.openLoop) {
        const msg = {
          open_loop: true,
          vel: this.velocity,
          site: this.site,
        };
        return new ROSLIB.Message(msg);
      }
      else {
        const msg = {
          open_loop: true,
          vel: 0,
          site: this.site,
        };
        return new ROSLIB.Message(msg);
      }
    },

    velocityScaleDecimal: function () {
      return this.velocityScale / 100;
    },
  },

  beforeUnmount: function () {
    this.openLoop = false;
    this.carousel_pub.publish(this.messageObject);

    window.clearInterval(interval);
  },

  created: function () {
    this.carousel_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "carousel_cmd",
      messageType: "mrover/Carousel",
    });

    interval = window.setInterval(() => {
      this.carousel_pub.publish(this.messageObject);
    }, updateRate * 1000);
  },

  methods: {
    toggleOpenLoop: function () {
      this.openLoop = !this.openLoop;
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
</style>
