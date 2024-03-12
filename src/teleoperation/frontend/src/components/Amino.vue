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
import LEDIndicator from "./LEDIndicator.vue";
import { mapActions } from 'vuex';

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
    isAmino: { //true = amino, false = benedict's
      type: Boolean,
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

      siteIndex: 0,

      autoShutdownEnabled: true,
      autoShutdownIntended: true,

    };
  },

  beforeUnmount: function () {
    window.clearInterval(interval);
  },

  created: function () {
    this.siteIndex = this.site.charCodeAt(0) - 65;
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    toggleHeater: function (id) {
      this.heaters[id].intended = !this.heaters[id].intended;
      this.sendHeaterRequest(id);
    },

    sendHeaterRequest: function (id) {
      this.heaters[id].enabled = !this.heaters[id].enabled;
      var heaterName = "n";
      if (this.isAmino) {
        heaterName = "b"
      }
      heaterName += id
      // window.setTimeout(() => {
        this.sendMessage({ type: "heaterEnable", enabled: this.heaters[id].enabled, heater: heaterName});
      // }, 250)
    },

    sendAutoShutdownCmd: function (enabled) {
      this.autoShutdownIntended = enabled;
    },

    capturePhoto() {
      // window.setTimeout(() => {
        this.sendMessage({ type: "takePanorama" }); //TODO: change to something other than takePanorama
      // }, 250)
    }
  }
};
</script>

<style scoped>
</style>