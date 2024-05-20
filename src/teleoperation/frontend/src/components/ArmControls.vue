<template>
  <div class='wrap'>
    <h2>Arm Controls</h2>
    <div class='controls-flex'>
      <h4>Mode</h4>
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
        <input v-model='mode' class='form-check-input' type='radio' id='manual' value='manual' />
        <label class='form-check-label' for='manual'>Manual</label>
      </div>
      <div class='form-check'>
        <input v-model='mode' class='form-check-input' type='radio' id='hybrid' value='hybrid' />
        <label class='form-check-label' for='hybrid'>Hybrid</label>
      </div>
    </div>
  </div>
</template>

<script lang='ts'>
import { defineComponent } from 'vue'
import { mapActions, mapState } from 'vuex'
import ToggleButton from './ToggleButton.vue'

const UPDATE_HZ = 20

export default defineComponent({
  components: {
    ToggleButton
  },
  data() {
    return {
      mode: 'disabled'
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  mounted: function() {
    document.addEventListener('keydown', this.keyDown)
  },

  created: function() {
    this.interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      const gamepad = gamepads.find(gamepad => gamepad && gamepad.id.includes('Microsoft'))
      if (!gamepad) return

      this.sendMessage({
        type: 'ra_controller',
        axes: gamepad.axes,
        buttons: gamepad.buttons.map(button => button.value)
      })

      this.sendMessage({
        type: 'ra_mode',
        mode: this.mode
      })
    }, 1000 / UPDATE_HZ)
  },

  beforeUnmount: function() {
    window.clearInterval(this.interval)
    document.removeEventListener('keydown', this.keyDown)
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    keyDown: function(event: { key: string }) {
      // Use the space bar as an e-stop
      if (event.key == ' ') {
        this.mode = 'disabled'
      }
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
