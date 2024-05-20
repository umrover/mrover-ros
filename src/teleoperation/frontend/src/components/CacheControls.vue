<template>
  <div class="wrap">
    <h3>Cache Controls</h3>
    <div class="controls-flex">
      <h5>Control Mode</h5>
      <div class="form-check">
        <input
          v-model="mode"
          class="form-check-input"
          type="radio"
          id="disabled"
          value="disabled"
        />
        <label class="form-check-label" for="disabled">Disabled</label>
      </div>
      <div class="form-check">
        <input v-model="mode" class="form-check-input" type="radio" id="throttle" value="throttle" />
        <label class="form-check-label" for="throttle">Throttle</label>
      </div>
    </div>
    <!--    <div class="limit-switch">-->
    <!--      <LimitSwitch-->
    <!--        :service_name="'cache_enable_limit_switches'"-->
    <!--        :display_name="'Cache Limit Switch'"-->
    <!--        :style="'width: 18%'"-->
    <!--      />-->
    <!--      <CalibrationCheckbox-->
    <!--        :name="'Calibrate Cache'"-->
    <!--        :topic_name="'calibrate_cache'"-->
    <!--        :style="'width: 20%'"-->
    <!--      ></CalibrationCheckbox>-->
    <!--    </div>-->
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import CalibrationCheckbox from './CalibrationCheckbox.vue'
import LimitSwitch from './LimitSwitch.vue'
import { mapActions, mapState } from 'vuex'

const UPDATE_HZ = 20

export default defineComponent({
  data() {
    return {
      mode: 'disabled',
      mappings: {
        w: 0,
        a: 1,
        s: 2,
        d: 3.
      },
      keys: Array(4).fill(0)
    }
  },

  components: {
    CalibrationCheckbox,
    LimitSwitch
  },

  beforeUnmount: function() {
    window.clearInterval(this.interval)
    document.removeEventListener('keyup', this.keyMonitorUp)
    document.removeEventListener('keydown', this.keyMonitorDown)
  },

  mounted: function() {
    document.addEventListener('keydown', this.keyMonitorDown)
    document.addEventListener('keyup', this.keyMonitorUp)
  },

  created: function() {
    this.interval = window.setInterval(() => {
      this.publish()
    }, 1000 / UPDATE_HZ)
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    keyMonitorDown: function(event: { key: string }) {
      const index = this.mappings[event.key.toLowerCase()]
      if (index === undefined) return

      this.keys[index] = 1
    },

    keyMonitorUp: function(event: { key: string }) {
      const index = this.mappings[event.key.toLowerCase()]
      if (index === undefined) return

      this.keys[index] = 0
    },

    publish: function() {
      this.sendMessage({
        type: 'cache_keyboard',
        axes: [],
        buttons: this.keys
      })

      this.sendMessage({
        type: 'cache_mode',
        mode: this.mode
      })
    }
  }
})
</script>

<style scoped>
.controls-flex {
  flex-wrap: wrap;
  display: flex;
  align-items: center;
  column-gap: 10px;
  padding-left: 10px;
  margin-bottom: 5px;
  margin-top: 5px;
}

.wrap {
  display: flex;
  flex-direction: column;
  gap: 5px;
}

.limit-switch {
  display: flex;
  flex-direction: row;
  align-items: center;
  padding-left: 10px;
}
</style>
