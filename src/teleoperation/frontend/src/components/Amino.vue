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
    <div class="capture sample picture">
      <button class="btn btn-primary btn-lg cutstom-btn" @click="capturePhoto()">Capture Photo</button>
    </div>
  </div>
</template>

<script>
import ToggleButton from "./ToggleButton.vue";
import ROSLIB from "roslib";
import LEDIndicator from "./LEDIndicator.vue";

let interval;

export default {
  components: {
    ToggleButton,
    LEDIndicator
  },

  props: {
    site: {
      type: String,
      required: true
    },
    siteIndex: {
      type: Number,
      required: true
    }
  },

  data() {
    return {
      heaters: [
        {
          enabled: false,
          intended: false,
          temp: 0,
          color: "grey"
        },
        {
          enabled: false,
          intended: false,
          temp: 0,
          color: "grey"
        },
        {
          enabled: false,
          intended: false,
          temp: 0,
          color: "grey"
        }
      ],

      autoShutdownEnabled: true,
      autoShutdownIntended: true,

      // Pubs and subs
      temp_sub: null,
      heater_status_sub: null,
      shutdown_status_sub: null,

      heater_service: null,
    };
  },

  beforeUnmount: function () {
    window.clearInterval(interval);
  },

  created: function () {
    this.temp_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "science/temperatures",
      messageType: "mrover/ScienceTemperature"
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
      messageType: "mrover/HeaterData"
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
      messageType: "std_msgs/Bool"
    });

    this.shutdown_status_sub.subscribe((msg) => {
      this.autoShutdownEnabled = msg.data;
    });

    this.heater_service = new ROSLIB.Service({
      ros: this.$ros,
      name: "change_heater_state",
      serviceType: "mrover/ChangeHeaterState"
    });

    // Get interval param
    let param = new ROSLIB.Param({
      ros: this.$ros,
      name: "science/heater_service_request_interval"
    });

    param.get((value) => {
      let interval_ms = value;
      // Send heater service request on interval for any activated heaters.
      interval = setInterval(() => {
        for (let i = 0; i < 3; i++) {
          if (this.heaters[i].intended) {
            this.sendHeaterRequest(i);
          }
        }
      }, interval_ms);
    });
  },

  methods: {
    toggleHeater: function (id) {
      this.heaters[id].intended = !this.heaters[id].intended;
      this.sendHeaterRequest(id);
    },

    sendHeaterRequest: function (id) {
      let request = new ROSLIB.ServiceRequest({
        device: id,
        enable: this.heaters[id].intended
      });

      this.heater_service.callService(request, (result) => {
        if (!result) {
          alert(`Toggling heater ${i} failed.`);
        }
      });
    },

    sendAutoShutdownCmd: function (enabled) {
      this.autoShutdownIntended = enabled;
      let autoShutdownServ = new ROSLIB.Service({
        ros: this.$ros,
        name: "change_heater_auto_shutoff_state",
        serviceType: "std_srvs/SetBool",
      });
      let request = new ROSLIB.ServiceRequest({
        data: this.autoShutdownIntended
      });
      autoShutdownServ.callService(request, (result) => {
        if (!result) {
          alert(`Toggling autoshutdown failed.`);
        }
      });
    }

    capturePhoto() {
      // TBD
    }
  }
};
</script>

<style scoped>
</style>