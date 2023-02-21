<template>
  <div>
    <h3>Raman</h3>
    <ToggleButton
      :current-state="ramanLaserState"
      label-enable-text="Raman Laser On"
      label-disable-text="Raman Laser Off"
      @change="toggleRamanLaser()"
    />
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
      ramanLaserState: false,
      ramanService: null,
    };
  },

  created: function () {
    this.ramanService = new ROSLIB.Service({
      ros: this.$ros,
      name: "change_raman_laser_state",
      serviceType: "mrover/ChangeDeviceState",
    });
  },

  methods: {
    toggleRamanLaser: function () {
      this.ramanLaserState = !this.ramanLaserState;
      let request = new ROSLIB.ServiceRequest({
        enable: this.ramanLaserState,
      });
      this.ramanService.callService(request, (result) => {
        if (!result) {
          this.ramanLaserState = !this.ramanLaserState;
          alert("Toggling Raman Laser failed.");
        }
      });
    },
  },
};
</script>

<style scoped></style>
