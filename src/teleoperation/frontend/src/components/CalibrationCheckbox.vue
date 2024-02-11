<template>
  <div class="calibration-wrapper">
    <Checkbox :name="name" @toggle="toggleCalibration"/>
    <span class="led">
      <LEDIndicator :connected="calibrated" :name="name" :show_name="false" />
    </span>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import Checkbox from './Checkbox.vue'
import LEDIndicator from './LEDIndicator.vue'
import { mapState, mapActions } from 'vuex'

export default defineComponent({
  components: {
    Checkbox,
    LEDIndicator
  },

  props: {
    name: {
      type: String,
      required: true
    },
    topic_name: {
      type: String,
      required: true
    },
  },

  data() {
    return {
      toggleEnabled: false,
      calibrated: false,
      interval: 0 as number
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == 'calibrate_motors') {
        if(this.toggleEnabled){
          if(Array.isArray(msg.result) && msg.result.length > 0){
              this.toggleEnabled = false
              for (var j = 0; j < msg.result.length; ++j) {
                alert('ESW cannot calibrate motor ' + msg.result[j])
              }
          }
          else if (typeof msg.result === "string"){
            this.toggleEnabled = false
            alert('ESW cannot calibrate motor ' + msg.result)
          }
        else this.calibrated = true
      }
      }
    },

    toggleEnabled: function (val) {
      // When the checkbox is toggled, publish a single false request to the calibrate service
      if (!val) {
        this.publishCalibrationMessage()
      }
    }
  },

  beforeUnmount: function () {
    clearInterval(this.interval)
    this.toggleEnabled = false
    this.publishCalibrationMessage()
  },

  created: function () {
    this.interval = setInterval(() => {
      if (!this.calibrated && this.toggleEnabled) {
        this.publishCalibrationMessage()
      }
    }, 200)
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
    toggleCalibration: function () {
      this.toggleEnabled = !this.toggleEnabled
    },
    publishCalibrationMessage: function () {
      this.sendMessage({ type: 'calibrate_motors', topic: this.topic_name })
    }
  }
})
</script>

<style>
.calibration-wrapper {
  padding: 1% 0 1% 0;
  display: flex;
  flex-direction: row;
}

.led {
  margin-left: 5%;
  display: block;
}
</style>
