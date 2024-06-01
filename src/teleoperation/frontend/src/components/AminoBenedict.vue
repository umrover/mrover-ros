<template>
  <div class="wrap">
    <h3 v-if="isAmino">Amino Test Controls</h3>
    <h3 v-else>Benedict's Test Controls</h3>
    <div class="box1 heaters">
      <ToggleButton
        id="heater"
        :current-state="heaters[site].enabled"
        :label-enable-text="'Heater ' + site"
        :label-disable-text="'Heater ' + site"
        @change="toggleHeater(site)"
      />
      <p :style="{ color: heaters[site].color }">
        Thermistor {{ site }}: {{ (heaters[site].temp).toFixed(2) }} C°
      </p>
    </div>
    <div class="comms heaterStatus">
      <LEDIndicator
        :connected="heaters[site].state"
        :name="'Heater ' + site + ' Status'"
        :show_name="true"
      />
    </div>
    <div class="box1 shutdown">
      <ToggleButton
        id="autoshutdown"
        :current-state="autoShutdownEnabled"
        :label-enable-text="'Auto Shutdown'"
        :label-disable-text="'Auto Shutdown'"
        @change="sendAutoShutdownCmd()"
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
import LEDIndicator from "./LEDIndicator.vue";
import { mapState, mapActions } from 'vuex';

let interval;

export default {
  components: {
    ToggleButton,
    LEDIndicator
  },

  props: {
    site: {
      type: Number,
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
          temp: 0,
          state: false,
          color: "grey"
        },
        {
          enabled: false,
          temp: 0,
          state: false,
          color: "grey"
        },
        {
          enabled: false,
          temp: 0,
          state: false,
          color: "grey"
        }
      ],

      autoShutdownEnabled: true,
    };
  },

  watch: {
    message(msg) {
      if (msg.type == 'auto_shutoff') {
        if (!msg.success) {
          this.autoShutdownEnabled = !this.autoShutdownEnabled;
          alert('Toggling Auto Shutdown failed.')
        }
      }
      else if (msg.type == 'thermistor') {
        if(this.site == 0) return;
        if (this.isAmino) {
          this.heaters[this.site].temp = msg.temps[this.site*2+1].temperature;
        }
        else {
          this.heaters[this.site].temp = msg.temps[this.site*2].temperature;
        }
      }
      else if(msg.type == 'heater_states') {
        if(this.site == 0) return;
        if (this.isAmino) {
          this.heaters[this.site].state = msg.state[this.site*2+1];
        }
        else {
          this.heaters[this.site].state = msg.state[this.site*2];
        }
      }
    },
  },

  beforeUnmount: function () {
    window.clearInterval(interval);
  },

  created: function () {
    interval = window.setInterval(() => {
      this.sendHeaterRequest(this.site);
    }, 100);    
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    toggleHeater: function (id) {
      this.heaters[id].enabled = !this.heaters[id].enabled;
      this.sendHeaterRequest(id);
    },

    sendHeaterRequest: function (id) {
      let heaterName = "b";
      if (this.isAmino) {
        heaterName = "n"
      }
      heaterName += id
      this.sendMessage({ type: "heater_enable", enabled: this.heaters[id].enabled, heater: heaterName});
    },

    sendAutoShutdownCmd: function () {
      this.autoShutdownEnabled = !this.autoShutdownEnabled;
      this.sendMessage({ type: "auto_shutoff", shutoff: this.autoShutdownEnabled });
    },

    capturePhoto() {
      this.sendMessage({ type: "capture_photo" });
    }
  }
};
</script>

<style scoped>
</style>