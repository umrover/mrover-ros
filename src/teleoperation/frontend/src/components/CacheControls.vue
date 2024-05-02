<template>
  <div class="wrap">
    <h3>Cache Controls</h3>
    <div class="controls-flex">
      <h5>Control Mode</h5>
      <div class="form-check">
        <input
          v-model="arm_mode"
          class="form-check-input"
          type="radio"
          id="dis"
          value="arm_disabled"
        />
        <label class="form-check-label" for="dis">Arm Disabled</label>
      </div>
      <div class="form-check">
        <input v-model="arm_mode" class="form-check-input" type="radio" id="pos" value="position" />
        <label class="form-check-label" for="pos">Position</label>
      </div>
      <div class="form-check">
        <input v-model="arm_mode" class="form-check-input" type="radio" id="vel" value="velocity" />
        <label class="form-check-label" for="vel">Velocity</label>
      </div>
      <div class="form-check">
        <input v-model="arm_mode" class="form-check-input" type="radio" id="thr" value="throttle" />
        <label class="form-check-label" for="thr">Throttle</label>
      </div>
    </div>
    <div class="controls-flex" v-if="arm_mode === 'position'">
      <div class="row">
        <div class="col" v-for="(joint, key) in temp_positions" :key="key">
          <label>{{ key }}</label>
          <input
            class="form-control"
            type="number"
            :min="joint.min"
            :max="joint.max"
            @input="validateInput(joint, $event)"
            v-model="joint.value"
          />
        </div>
      </div>
      <button class="btn btn-primary my-2" @click="submit_positions">Submit</button>
    </div>
    <div class="limit-switch">
      <LimitSwitch
        :service_name="'cache_enable_limit_switches'"
        :display_name="'Cache Limit Switch'"
        :style="'width: 18%'"
      />
      <CalibrationCheckbox
        :name="'Calibrate Cache'"
        :topic_name="'calibrate_cache'"
        :style="'width: 20%'"
      ></CalibrationCheckbox>
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import CalibrationCheckbox from './CalibrationCheckbox.vue'
import LimitSwitch from './LimitSwitch.vue'
import { mapActions, mapState } from 'vuex'

// Update rate in seconds
const updateRate = 0.1
let interval: number|undefined

export default defineComponent({
  data() {
    return {
      arm_mode: 'arm_disabled',
      temp_positions: {
        cache: {
          value: 0,
          min: -100,
          max: 100
        }
      },
      positions:[],
      send_positions: false, // Only send after submit is clicked for the first time

      inputData: {
        a_key: 0,
        s_key: 0,
      }
    }
  },

  components: {
    CalibrationCheckbox,
    LimitSwitch
  },

  beforeUnmount: function () {
    window.clearInterval(interval)
    document.removeEventListener('keyup', this.keyMonitorUp)
    document.removeEventListener('keydown', this.keyMonitorDown)
  },

  mounted: function() {
    document.addEventListener('keydown', this.keyMonitorDown);
    document.addEventListener('keyup', this.keyMonitorUp);
  },

  created: function () {
    interval = window.setInterval(() => {
      if (this.send_positions) {
        this.publish()
      } else if (this.arm_mode !== "position") {
        this.publish()
      }
    }, updateRate * 1000)
  },

  watch: {
    arm_mode(newMode) {
      if (newMode !== 'position') {
        this.positions = []
        this.send_positions = false
      }
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
    validateInput(joint: any, event: any) {
      if (event.target.value < joint.min) {
        joint.value = joint.min
      } else if (event.target.value > joint.max) {
        joint.value = joint.max
      }
    },

    submit_positions: function () {
      this.positions = Object.values(this.temp_positions).map(
        (obj: any) => (Number(obj.value) * Math.PI) / 180
      )
      this.send_positions = true
      this.publish()
    },

    keyMonitorDown: function (event: { key: string }) {
      if (event.key.toLowerCase() == 'a') {
        this.inputData.a_key = 1
      } else if (event.key.toLowerCase() == 's') {
        this.inputData.s_key = 1
      }

      this.publish()
    },

    // when a key is released, sets input for that key as 0
    keyMonitorUp: function (event: { key: string }) {
      if (event.key.toLowerCase() == 'a') {
        this.inputData.a_key = 0
      } else if (event.key.toLowerCase() == 's') {
        this.inputData.s_key = 0
      }

      this.publish()
    },

    publish: function () {
      if (this.arm_mode != 'arm_disabled') {
        this.sendMessage({
          type: 'cache_values',
          input: this.inputData.d_key - this.inputData.a_key,
          arm_mode: this.arm_mode,
          positions: this.positions
        })
      }
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
