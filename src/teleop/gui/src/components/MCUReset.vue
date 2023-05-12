<template>
  <div class="wrap">
    <ToggleButton
      id="mcu_reset"
      :current-state="reset"
      label-enable-text=" resetting..."
      label-disable-text=" reset"
      @change="toggleReset()"
    />
    <div class="status">
      <LEDIndicator
        :connected="mcuActive"
        :name="'MCU'"
        :show_name="false"
      />
      <h3 class="header">MCU</h3>
    </div>
    <ToggleButton
      id="mcu_auton_reset"
      :current-state="autonReset"
      label-enable-text=" auton reset enabled"
      label-disable-text=" auton reset disabled"
      @change="toggleAutonReset()"
    />
  </div>
</template>

<script>
import LEDIndicator from "./LEDIndicator.vue";
import ToggleButton from "./ToggleButton.vue";
import ROSLIB from "roslib";

const RESET_TIMEOUT_S = 5;

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
      resetTimeoutID: 0,

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
      serviceType: "std_srvs/SetBool",
    });

    this.autonResetService = new ROSLIB.Service({
      ros: this.$ros,
      name: "reset_mcu_autonomously",
      serviceType: "std_srvs/SetBool",
    });
  },

  methods: {
    toggleReset: function () {
      if (!this.reset) {
        let confirmed = confirm("Are you sure you want to reset the Science MCU?");

        if (!confirmed) {
          return;
        }

        this.reset = true;

        let request = new ROSLIB.ServiceRequest({
          data: true,
        });

        this.resetTimeoutID = setTimeout(() => {
          alert("Attempt to reset Science MCU timed out.");
          this.reset = false;
        }, RESET_TIMEOUT_S * 1000);

        this.resetService.callService(request, (result) => {
          this.reset = false;

          clearTimeout(this.resetTimeoutID);

          if (!result.success) {
            alert("Resetting the Science MCU failed.");
          }
          else {
            alert("Success!");
          }
        });
      }
    },

    toggleAutonReset: function () {
      let request = new ROSLIB.ServiceRequest({
        data: !this.autonReset,
      });


      this.autonResetService.callService(request, (result) => {
        if (result.success) {
          this.autonReset = !this.autonReset;
        }
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

.header {
  margin-left: 4px;
}

.status {
  display: flex;
  align-items: center;
  height: 100%;
  margin: 15px;
}

</style>