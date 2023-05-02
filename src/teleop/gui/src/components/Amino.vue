<template>
  <div class="wrap">
    <h3>Amino Test Controls</h3>
    <div class="box1 heaters">
      <ToggleButton
        id="heater"
        :current-state="heaters[siteIndex].intended"
        :label-enable-text="'Heater ' + site + ' Intended'"
        :label-disable-text="'Heater ' + site + ' Intended'"
        @change="toggleHeater(siteIndex)"
      />
      <p :style="{ color: heaters[siteIndex].color }">
        Thermistor {{ site }}: {{ heaters[siteIndex].temp.toFixed(2) }} C°
      </p>
    </div>
    <div class="comms heaterStatus">
      <LEDIndicator
        :connected="heaters[siteIndex].enabled"
        :name="'Heater ' + site + ' Status'"
        :show_name="true"
      />
    </div>
    <div class="box1 shutdown">
      <ToggleButton
        id="autoshutdown"
        :current-state="autoShutdownIntended"
        :label-enable-text="'Auto Shutdown Intended'"
        :label-disable-text="'Auto Shutdown Intended'"
        @change="sendAutoShutdownCmd(!autoShutdownIntended)"
      />
    </div>
    <div class="comms shutdownStatus">
      <LEDIndicator
        :connected="autoShutdownEnabled"
        :name="'Auto Shutdown Status'"
        :show_name="true"
      />
    </div>
  </div>
</template>

<script>
import ToggleButton from "./ToggleButton.vue";
import ROSLIB from "roslib";
import LEDIndicator from "./LEDIndicator.vue";

export default {
  components: {
    ToggleButton,
    LEDIndicator,
  },

  props: {
    site: {
      type: String,
      required: true,
    },
    siteIndex: {
      type: Number,
      required: true,
    },
  },
  data() {
    return {
      heaters: [
        {
          enabled: false,
          intended: false,
          temp: 0,
          color: "grey",
        },
        {
          enabled: false,
          intended: false,
          temp: 0,
          color: "grey",
        },
        {
          enabled: false,
          intended: false,
          temp: 0,
          color: "grey",
        },
      ],

      autoShutdownEnabled: true,
      autoShutdownIntended: true,

      // Pubs and subs
      temp_sub: null,
      heater_status_sub: null,
      shutdown_status_sub: null,
    };
  },

  created: function () {
    this.temp_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "science/temperatures",
      messageType: "mrover/ScienceTemperature",
    });

    this.temp_sub.subscribe((msg) => {
      // Callback for temp_sub
      for (let i = 0; i < 3; i++) {
        this.heaters[i].temp = msg.temperatures[i];
      }
    });

    this.heater_status_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "science/heater_state_data",
      messageType: "mrover/HeaterData",
    });

    this.heater_status_sub.subscribe((msg) => {
      // Callback for heater_status_sub
      for (let i = 0; i < 3; i++) {
        this.heaters[i].enabled = msg.state[i];
      }
    });

    this.shutdown_status_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "science/heater_auto_shutoff_state_data",
      messageType: "std_msgs/Bool",
    });

    this.shutdown_status_sub.subscribe((msg) => {
      this.autoShutdownEnabled = msg.data;
    });
  },

  methods: {
    toggleHeater: function (id) {
      this.heaters[id].intended = !this.heaters[id].intended;
      let toggleHeaterServ = new ROSLIB.Service({
        ros: this.$ros,
        name: "change_heater_state",
        serviceType: "mrover/ChangeHeaterState",
      });
      let request = new ROSLIB.ServiceRequest({
        device: id,
        enable: this.heaters[id].intended,
      });
      toggleHeaterServ.callService(request, (result) => {
        if (!result) {
          alert(`Toggling heater ${id} failed.`);
        }
      });
    },

    sendAutoShutdownCmd: function (enabled) {
      this.autoShutdownIntended = enabled;
      let autoShutdownServ = new ROSLIB.Service({
        ros: this.$ros,
        name: "change_heater_auto_shutoff_state",
        serviceType: "mrover/ChangeHeaterAutoShutoffState",
      });
      let request = new ROSLIB.ServiceRequest({
        enable: this.autoShutdownIntended,
      });
      autoShutdownServ.callService(request, (result) => {
        if (!result) {
          alert(`Toggling autoshutdown failed.`);
        }
      });
    },
  },
};
</script>

<style scoped>
.wrap {
  display: grid;
  grid-gap: 5px;
  grid-template-columns: auto auto;
  grid-template-rows: auto auto auto auto;
  grid-template-areas:
    "title ."
    "select select"
    "heaters heaterStatus"
    "shutdown shutdownStatus";
  height: auto;
}

.select {
  grid-area: select;
}

.heaters {
  grid-area: heaters;
}

.heaterStatus {
  grid-area: heaterStatus;
}

.shutdown {
  grid-area: shutdown;
}

.shutdownStatus {
  grid-area: shutdownStatus;
}

.box1 {
  text-align: left;
  vertical-align: top;
  display: inline-block;
}

.comms {
  display: flex;
  flex-direction: column;
  align-items: flex-start;
}

.comms * {
  margin-top: 2px;
  margin-bottom: 2px;
}
</style>
