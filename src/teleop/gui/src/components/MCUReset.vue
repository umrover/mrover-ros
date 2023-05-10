<template>
  <div class="wrap">
    <div>
      <ToggleButton
        id="mcu_reset"
        :current-state="reset"
        label-enable-text="MCU resetting..."
        label-disable-text="MCU active"
        @change="toggle()"
      />
    </div>
  </div>
</template>

<script>
import ToggleButton from "./ToggleButton.vue";
import ROSLIB from "roslib";

const RESET_TIMEOUT_S = 8;

export default {
  components: {
    ToggleButton
  },

  data() {
    return {
      reset: false,
      timeoutID: 0,

      // Service Client
      resetService: null,
    };
  },

  created: function () {
    this.resetService = new ROSLIB.Service({
      ros: this.$ros,
      name: "mcu_board_reset",
      serviceType: "mrover/EnableDevice",
    });
  },

  methods: {
    toggle: function () {
      if (!this.reset) {
        let confirmed = confirm("Are you sure you want to reset the Science MCU?\nIt will be inactive for ~30 seconds.");

        if (!confirmed) {
          return;
        }

        this.reset = true;

        let request = new ROSLIB.ServiceRequest({
          name: "mcu_board_reset",
          enable: true,
        });

        this.timeoutID = setTimeout(() => {
          alert("Attempt to reset Science MCU timed out.");
          this.reset = false;
        }, RESET_TIMEOUT_S * 1000);

        this.resetService.callService(request, (result) => {
          this.reset = false;

          if (!result.success) {
            alert("Resetting the Science MCU failed.");
          }
          else {
            alert("Success! Please reset the Pi node once the MCU is back online!!");
          }

          clearTimeout(this.timeoutID);
        });
      }
    }
  }



};

</script>

<style scoped>
.wrap {
  display: flex;
  align-items: center;
  height: 100%;
}
</style>