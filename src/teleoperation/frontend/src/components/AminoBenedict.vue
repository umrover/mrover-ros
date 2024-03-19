<template>
  <div class="wrap">
    <h3 v-if="isAmino">Amino Test Controls</h3>
    <h3 v-else>Benedict's Test Controls</h3>
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
        :connected="autoShutdownIntended"
        :name="'Auto Shutdown Status'"
        :show_name="true"
      />
    </div>
    <div class="capture sample picture">
      <div class="d-flex justify-content-end">
        <button class="btn btn-primary btn-lg cutstom-btn" @click="capturePhoto()">Capture Photo</button>
      </div>
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

  watch: {
    message(msg) {
      if (msg.type == 'heaterEnable') {
        if (!msg.result) {
          this.heaters[id].enabled = !this.heaters[id].enabled
          alert('Toggling Heater Enable failed.')
        }
      }
      else if (msg.type == 'autoShutoff') {
        if (!msg.result) {
          autoShutdownIntended = !autoShutdownIntended
          alert('Toggling Auto Shutdown failed.')
        }
      }
      else if (msg.type == 'thermistor') {
        var heaterID = 'b'
        if (isAmino) {
          heaterID = 'n'
        }
        if (heaterID == 'n') {
          this.heathers[id].temp = msg.thermistor_data[id*2]
        }
        else if (heaterID == 'b') {
          this.heathers[id].temp = msg.thermistor_data[(id*2)+1]
        }
      }
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
      var heaterName = "b";
      if (this.isAmino) {
        heaterName = "n"
      }
      heaterName += id
      this.sendMessage({ type: "heaterEnable", enabled: this.heaters[id].enabled, heater: heaterName});
    },

    sendAutoShutdownCmd: function (enabled) {
      this.autoShutdownIntended = enabled;
      this.sendMessage({ type: "autoShutoff", shutoff: this.autoShutdownIntended });
    },

    capturePhoto() {
      this.sendMessage({ type: "capturePhoto" }); 
    }
  }
};
</script>

<style scoped>
</style>