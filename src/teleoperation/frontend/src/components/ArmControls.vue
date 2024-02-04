<template>
  <div class="wrap">
    <h2>Arm Controls</h2>
    <div class="controls-flex">
      <h4>Arm mode</h4>
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
        <input v-model="arm_mode" class="form-check-input" type="radio" id="ik" value="ik" />
        <label class="form-check-label" for="ik">IK</label>
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
      <div class="col text-center">
        <button class="btn btn-primary" @click="submit_positions">Submit</button>
      </div>
    </div>
    <div class="controls-flex">
      <h4>Misc. Controls</h4>
      <ToggleButton
        id="arm_laser"
        :current-state="laser_enabled"
        label-enable-text="Arm Laser On"
        label-disable-text="Arm Laser Off"
        @change="toggleArmLaser()"
      />
      <div class="limit-switch">
        <h4>Limit Switches</h4>
        <LimitSwitch :name="'All Switches'" :switch_name="'all_ra'" />
      </div>
    </div>
    <div class="controls-flex">
      <h4>Calibration</h4>
      <CalibrationCheckbox name="All Joints Calibration" topic_name = "all_ra" />
      <MotorAdjust
        v-if="arm_mode == 'position'"
        :motors="[
          { esw_name: 'joint_a', display_name: 'Joint A' },
          { esw_name: 'joint_b', display_name: 'Joint B' },
          { esw_name: 'joint_c', display_name: 'Joint C' },
          { esw_name: 'joint_de_pitch', display_name: 'Joint DE Pitch' },
          { esw_name: 'joint_de_roll', display_name: 'Joint DE Yaw' }
        ]"
      />
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import { mapActions } from 'vuex'
import ToggleButton from './ToggleButton.vue'
import CalibrationCheckbox from './CalibrationCheckbox.vue'
import MotorAdjust from './MotorAdjust.vue'
import LimitSwitch from './LimitSwitch.vue'

// In seconds
const updateRate = 0.1
let interval: number | undefined

export default defineComponent({
  components: {
    ToggleButton,
    CalibrationCheckbox,
    MotorAdjust,
    LimitSwitch
  },
  data() {
    return {
      websocket: new WebSocket('ws://localhost:8000/ws/gui'),
      arm_mode: 'arm_disabled',
      joints_array: [false, false, false, false, false, false],
      laser_enabled: false,
      temp_positions: {
        /* Positions in degrees! */
        /* Joint A, allen_key and gripper don't need positioning */
        joint_b: {
          value: 0,
          min: -45,
          max: 0
        },
        joint_c: {
          value: 0,
          min: -100,
          max: 120
        },
        joint_de_pitch: {
          value: 0,
          min: -135,
          max: 135
        },
        joint_de_roll: {
          value: 0,
          min: -135,
          max: 135
        }
      },
      positions: []
    }
  },

  created: function () {
    this.websocket.onmessage = (event) => {
      const msg = JSON.parse(event.data)
      if (msg.type == 'laser_service') {
        if (!msg.result) {
          this.laser_enabled = !this.laser_enabled
          alert('Toggling Arm Laser failed.')
        }
      }
    }
    interval = window.setInterval(() => {
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
          type: 'arm_values',
          axes: axes,
          buttons: buttons,
          arm_mode: arm_mode,
          positions: positions
        })
      }
    },
    toggleArmLaser: function () {
      this.laser_enabled = !this.laser_enabled
      this.sendMessage({ type: 'laser_service', data: this.laser_enabled })
    },

    submit_positions: function () {
      //converts to radians
      this.positions = Object.values(this.temp_positions).map(
        (obj) => (Number(obj.value) * Math.PI) / 180
      )
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
