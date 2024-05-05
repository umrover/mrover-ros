<template>
  <div class="wrap">
    <h2>SA Arm Controls</h2>
    <div class="controls-flex">
      <h4>Arm mode</h4>
      <!-- Make opposite option disappear so that we cannot select both -->
      <!-- Change to radio buttons in the future -->
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
      <button class="btn btn-primary mx-auto my-2" @click="submit_positions">Submit</button>
    </div>
    <div class="controls-flex">
      <button class="btn btn-primary" @click="zero">Zero Z</button>
      <p>SA Z Position: {{ z_position }} inches</p>
      <MotorAdjust
        v-if="arm_mode == 'position'"
        :options="[
          { esw_name: 'sa_x', display_name: 'X' },
          { esw_name: 'sa_y', display_name: 'Y' },
          { esw_name: 'sa_z', display_name: 'Z' },
          { esw_name: 'sampler', display_name: 'Sampler' },
          { esw_name: 'sensor_actuator', display_name: 'Sensor Actuator' }
        ]"
      />
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import { mapActions, mapState } from 'vuex'
import CalibrationCheckbox from './CalibrationCheckbox.vue'
import MotorAdjust from './MotorAdjust.vue'

// In seconds
const updateRate = 0.01
const metersToInches = 39.3701
let interval: number | undefined

export default defineComponent({
  components: {
    CalibrationCheckbox,
    MotorAdjust,
  },
  data() {
    return {
      arm_mode: 'arm_disabled',
      laser_enabled: false,
      /* Positions in degrees! */
      temp_positions: {
        sa_x: {
          value: 0,
          min: -100,
          max: 100
        },
        sa_y: {
          value: 0,
          min: -100,
          max: 100
        },
        sa_z: {
          value: 0,
          min: -100,
          max: 100
        },
        sampler: {
          value: 0,
          min: -100,
          max: 100
        },
        sensor_actuator: {
          value: 0,
          min: -100,
          max: 100
        }
      },
      positions: [],
      send_positions: false, // Only send after submit is clicked for the first time,
      z_position: 0
    }
  },

  created: function () {
    interval = window.setInterval(() => {
      if (this.send_positions) {
        this.publishJoystickMessage([], [], this.arm_mode, this.positions)
      } else if (this.arm_mode !== "position") {
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

              this.publishJoystickMessage(gamepad.axes, buttons, this.arm_mode, [])
            }
          }
        }
      }
    }, updateRate * 1000)
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == 'laser_service') {
        if (!msg.result) {
          this.laser_enabled = !this.laser_enabled
          alert('Toggling Arm Laser failed.')
        }
      }
      else if (msg.type == 'sa_z') {
        this.z_position = msg.sa_z * metersToInches
      }
    },
    arm_mode(newMode) {
      if (newMode !== 'position') {
        this.positions = []
        this.send_positions = false
      }
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    validateInput: function (joint, event) {
      if (event.target.value < joint.min) {
        event.target.value = joint.min
      } else if (event.target.value > joint.max) {
        event.target.value = joint.max
      }
      joint.value = event.target.value
    },

    publishJoystickMessage: function (axes: any, buttons: any, arm_mode: any, positions: any) {
      if (arm_mode != 'arm_disabled') {
        this.sendMessage({
          type: 'sa_arm_values',
          axes: axes,
          buttons: buttons,
          arm_mode: arm_mode,
          positions: positions
        })
      }
    },

    submit_positions: function () {
      //converts to radians
      this.positions = Object.values(this.temp_positions).map(
        (obj) => (Number(obj.value) * Math.PI) / 180
      )
      this.send_positions = true
      this.publishJoystickMessage([], [], this.arm_mode, this.positions)
    },

    zero: function() {
      this.sendMessage({type: "arm_adjust", name: "sa_z", value: 0})
    }
  }
})
</script>

<style scoped>
.wrap {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-items: center;
  width: 100%;
  height: auto;
}

.wrap h2 h4 {
  margin: 0;
  padding: 0;
  font-size: 1.5em;
  font-weight: bold;
  text-align: center;
  width: 100%;
  padding-top: 5px;
  padding-bottom: 5px;
}

.controls-flex {
  flex-wrap: wrap;
  display: flex;
  align-items: center;
  width: 100%;
  column-gap: 20px;
  padding-left: 10px;
  margin-bottom: 5px;
  margin-top: 5px;
}

.limit-switch {
  display: flex;
  flex-direction: column;
  align-items: center;
}

.limit-switch h4 {
  margin-bottom: 5px;
}

.position-box {
  width: 50px;
}
</style>
