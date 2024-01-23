<template>
  <div class="calibration-wrapper">
    <Checkbox :name="name" @toggle="toggleCalibration"> </Checkbox>
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
    joint_name: {
      type: String,
      required: true
    },
    calibrate_topic: {
      type: String,
      required: true
    }
  },

  data() {
    return {
      socket: null,
      toggleEnabled: false,
      calibrated: false,
      calibrate_service: null,
      calibrate_sub: null,
      interval: 0 as number
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == 'calibration_status') {
        for (var i = 0; i < msg.names.length; ++i) {
          if (msg.names[i] == this.joint_name) {
            this.calibrated = msg.calibrated[i]
            break
          }
        }
      } else if (msg.type == 'calibrate_service') {
        if (!msg.result) {
          this.toggleEnabled = false
          alert('ESW cannot calibrate this motor')
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
      this.sendMessage({ type: 'calibrate_service', data: this.toggleEnabled })
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
