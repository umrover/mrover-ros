<template>
<div class="wrap">
  <div class="title">
    <h3>Amino Test Controls</h3>
  </div>
  <div class="box1 heaters">
    <ToggleButton id="heater" v-bind:currentState="heaters[siteIndex].intended" :labelEnableText="'Heater '+(site)+' Intended'" :labelDisableText="'Heater '+(site)+' Intended'" v-on:change="toggleHeater(siteIndex)"/>
    <p v-bind:style="{color: heaters[siteIndex].color}">Thermistor {{site}}: {{heaters[siteIndex].temp.toFixed(2)}} C°</p>
  </div>
  <div class="comms heaterStatus">
    <!-- <CommIndicator v-bind:connected="heaters[site].enabled" v-bind:name="'Heater '+(site)+' Status'"/> -->
  </div>
  <div class="box1 shutdown">
    <ToggleButton id="autoshutdown" v-bind:currentState="autoShutdownIntended" :labelEnableText="'Auto Shutdown Intended'" :labelDisableText="'Auto Shutdown Intended'" v-on:change="sendAutoShutdownCmd(!autoShutdownIntended)"/>
  </div>
  <div class="comms shutdownStatus">
    <!-- <CommIndicator v-bind:connected="autoShutdownEnabled" v-bind:name="'Auto Shutdown Status'"/> -->
  </div>
</div>  
</template>

<script>
import ToggleButton from './ToggleButton.vue'
// import CommIndicator from './CommIndicator.vue'

export default {
  data () {
    return {
      siteIndex: 0,
      heaters: [
        {
          enabled: false,
          intended: false,
          temp: 0,
          color: 'grey'
        },
        {
          enabled: false,
          intended: false,
          temp: 0,
          color: 'grey'
        },
        {
          enabled: false,
          intended: false,
          temp: 0,
          color: 'grey'
        }
      ],

      autoShutdownEnabled: true,
      autoShutdownIntended: true
    }
  },

  props: {
    site: {
        type: String,
        required: true,
    }
  },

  created: function () {
    
  },

  components: {
    ToggleButton,
  },

  methods: {
    toggleHeater: function(id) {
      this.heaters[id].intended = !this.heaters[id].intended
    },

    sendAutoShutdownCmd: function(enabled) {
      this.autoShutdownIntended = enabled
    }
  },

  watch: {
    site(newVal, oldVal) {
        if (newVal === "A") {
            this.siteIndex = 0
        } else if (newVal === "B") {
            this.siteIndex = 1
        } else {
            this.siteIndex = 2
        }
    }
  }
}
</script>

<style scoped>
  .wrap {
    display: grid;
    grid-gap: 5px;
    grid-template-columns: auto auto;
    grid-template-rows: auto auto auto auto;
    grid-template-areas:  "title ."
                          "select select"
                          "heaters heaterStatus"
                          "shutdown shutdownStatus";
    height: auto;
  }
  .title {
    grid-area: title;
    text-align: left;
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