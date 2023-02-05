<template>
  <div class="wrap">
    <div>
      <h3>End Effector UV</h3>
    </div>

    <div>
      <ToggleButton
        id="uv_end_effector"
        :current-state="endEffectorUVActive"
        label-enable-text="End Effector UV On"
        label-disable-text="End Effector UV Off"
        @change="toggleEndEffectorUV()"
      />
    </div>
    <br />
    <div>
      <ToggleButton
        id="shutdown"
        :current-state="shutdownActive"
        label-enable-text="UV Auto shutoff On"
        label-disable-text="UV Auto shutoff Off"
        @change="switchShutdown()"
      />
    </div>
  </div>
</template>

<script>
import ToggleButton from "./ToggleButton.vue";
import ROSLIB from "roslib";

export default {
  components: {
    ToggleButton,
  },
  data() {
    return {
      endEffectorUVActive: false,
      shutdownActive: true,
      timeoutID: 0,
      scoopLimitActive: true,

      // Pubs and subs
      uvService: null,
    };
  },

  watch: {
    endEffectorUVActive: function (newVal, oldVal) {
      let request = new ROSLIB.ServiceRequest({
        enable: newVal,
      });
      this.uvService.callService(request, (result) => {
        if (!result) {
          this.endEffectorUVActive = oldVal;
          alert("Toggling End Effector UV failed.");
        }
      });
    },
  },

  created: function () {
    this.uvService = new ROSLIB.Service({
      ros: this.$ros,
      name: "change_uv_led_end_effector_state",
      serviceType: "mrover/ChangeDeviceState",
    });
  },

  methods: {
    switchShutdown: function () {
      this.shutdownActive = !this.shutdownActive;
    },

    toggleEndEffectorUV: function () {
      this.endEffectorUVActive = !this.endEffectorUVActive;
      if (this.endEffectorUVActive) {
        this.timeoutID = setTimeout(() => {
          if (this.endEffectorUVActive && this.shutdownActive) {
            this.endEffectorUVActive = false;
          }
        }, 2 * 60000); // 2 minutes
      } else {
        clearTimeout(this.timeoutID);
      }
    },

    toggleLimit: function () {
      this.scoopLimitActive = !this.scoopLimitActive;
    },
  },
};
</script>

<style scoped>
.wrap {
  display: inline-block;
  align-content: center;
}
</style>
