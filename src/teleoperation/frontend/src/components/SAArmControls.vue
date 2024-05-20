<template>
  <div class='wrap'>
    <h2>SA Arm Controls</h2>
    <div class='controls-flex'>
      <h4>Arm Mode</h4>
      <div class='form-check'>
        <input
          v-model='mode'
          class='form-check-input'
          type='radio'
          id='disabled'
          value='disabled'
        />
        <label class='form-check-label' for='disabled'>Disabled</label>
      </div>
      <div class='form-check'>
        <input v-model='mode' class='form-check-input' type='radio' id='thr' value='throttle' />
        <label class='form-check-label' for='thr'>Throttle</label>
      </div>
    </div>
    <div class='controls-flex'>
      <button class='btn btn-primary' @click='zero'>Zero Z</button>
      <p>SA Z Position: {{ z_position }} inches</p>
      <!--      <MotorAdjust-->
      <!--        v-if="mode == 'position'"-->
      <!--        :options="[-->
      <!--          { esw_name: 'sa_x', display_name: 'X' },-->
      <!--          { esw_name: 'sa_y', display_name: 'Y' },-->
      <!--          { esw_name: 'sa_z', display_name: 'Z' },-->
      <!--          { esw_name: 'sampler', display_name: 'Sampler' },-->
      <!--          { esw_name: 'sensor_actuator', display_name: 'Sensor Actuator' }-->
      <!--        ]"-->
      <!--      />-->
    </div>
  </div>
</template>

<script lang='ts'>
import { defineComponent } from 'vue'
import { mapActions, mapState } from 'vuex'
import CalibrationCheckbox from './CalibrationCheckbox.vue'
import MotorAdjust from './MotorAdjust.vue'

const UPDATE_HZ = 20

export default defineComponent({
  components: {
    CalibrationCheckbox,
    MotorAdjust
  },
  data() {
    return {
      mode: 'disabled',
      z_position: 0
    }
  },

  created: function() {
    this.interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      const gamepad = gamepads.find(gamepad => gamepad && gamepad.id.includes('Microsoft'))
      if (!gamepad) return

      this.sendMessage({
        type: 'controller',
        axes: gamepad.axes,
        buttons: gamepad.buttons.map(button => button.value)
      })

      this.sendMessage({
        type: 'sa_mode',
        mode: this.mode
      })
    }, 1000 / UPDATE_HZ)
  },

  beforeUnmount() {
    window.clearInterval(this.interval)
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      // TODO: get SA Z
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    zero: function() {
      this.sendMessage({ type: 'sa_adjust', name: 'sa_z', value: 0 })
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
  font-size: 1.5em;
  font-weight: bold;
  text-align: center;
  width: 100%;
  padding: 5px 0;
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

</style>
