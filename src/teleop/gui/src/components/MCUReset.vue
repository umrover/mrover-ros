<template>
  <div class="wrap">
    <LEDIndicator
      :connected="mcuActive"
      :name="'MCU'"
      :show_name="false"
    />
    <ToggleButton
      id="mcu_reset"
      :current-state="reset"
      label-enable-text=" MCU resetting..."
      label-disable-text=" MCU reset"
      @change="toggleReset()"
    />
    <ToggleButton
      id="mcu_auton_reset"
      :current-state="autonReset"
      label-enable-text=" auton reset enabled"
      label-disable-text=" auton reset disabledt"
      @change="toggleAutonReset()"
    />
  </div>
</template>

<script>
import LEDIndicator from "./LEDIndicator.vue";
import ToggleButton from "./ToggleButton.vue";
import ROSLIB from "roslib";

const RESET_TIMEOUT_S = 8;

export default {
  components: {
    LEDIndicator,
    ToggleButton
  },

  data() {
    return {
      mcuActive: false,
      activeSub: null,

      reset: false,
      timeoutID: 0,

      // Service Clients
      resetService: null,

      autonReset: true,
      autonResetService: null,
    };
  },

  created: function () {
    this.activeSub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "science_mcu_active",
      messageType: "std_msgs/Bool"
    });
    this.activeSub.subscribe((msg) => {
      this.mcuActive = msg.data;
    })

    this.resetService = new ROSLIB.Service({
      ros: this.$ros,
      name: "mcu_board_reset",
      serviceType: "mrover/EnableDevice",
    });

    this.autonResetService = new ROSLIB.Service({
      ros: this.$ros,
      name: "reset_mcu_autonomously",
      serviceType: "mrover/EnableDevice",
    });
  },

  methods: {
    toggleReset: function () {
      if (!this.reset) {
        let confirmed = confirm("Are you sure you want to reset the Science MCU?\nIt will be inactive for ~30 seconds.");

        if (!confirmed) {
          return;
        }

        this.reset = true;

        let request = new ROSLIB.ServiceRequest({
          name: "",
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
    },


    toggleAutonReset: function () {
      this.autonResetService.callService(request, (result) => {
        this.autonReset = !this.autonReset;
      });
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