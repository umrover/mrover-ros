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
      interval: null as number | null
    }
  },
  created: function () {
    this.interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      for (let i = 0; i < 4; i++) {
        const gamepad = gamepads[i]
        if (gamepad) {
          // Microsoft and Xbox for old Xbox 360 controllers
          // X-Box for new PowerA Xbox One controllers
          if (
            gamepad.id.includes('Microsoft') ||
            gamepad.id.includes('Xbox') ||
            gamepad.id.includes('X-Box')
          ) {
            let buttons = gamepad.buttons.map((button) => {
              return button.value
            })

            this.publishJoystickMessage(gamepad.axes, buttons, this.arm_mode, this.positions)
          }
        }
      }
    }, updateRate * 1000)
  },
  components: {
    CalibrationCheckbox,
    LimitSwitch
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
      // TODO: Redo position logic to be similar to SAArmControls.vue so that they will be sent repeatedly on the gamepad interval
      //converts to radians

      this.positions = Object.values(this.temp_positions).map(
        (obj: any) => (Number(obj.value) * Math.PI) / 180
      )




      //   this.sendMessage({
      //       type: 'cache_values',
      //       axes: axes,
      //       buttons: buttons,
      //       arm_mode: "position",
      //       positions: positions
      //     })
    },
    publishJoystickMessage: function (axes: any, buttons: any, arm_mode: any, positions: any) {
      if (arm_mode != 'arm_disabled') {
        this.sendMessage({
          type: 'cache_values',
          axes: axes,
          buttons: buttons,
          arm_mode: arm_mode,
          positions: positions
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
