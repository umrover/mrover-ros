<template>
  <div class='wrap'>
    <div v-show='false' id='key'>
      <input @keydown='keyMonitorDown' />
      <input @keyup='keyMonitorUp' />
    </div>
  </div>
</template>

<script lang='ts'>
import { mapActions } from 'vuex'

const UPDATE_HZ = 20

export default {
  data() {
    return {
      rotation_pwr: 1,
      up_down_pwr: 1,

      keyboard_pub: null,

      inputData: {
        w_key: 0,
        a_key: 0,
        s_key: 0,
        d_key: 0
      }
    }
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
    // When a key is being pressed down, set the power level.
    // Ignore keys that are already pressed to avoid spamming when holding values.
    keyMonitorDown: function(event: { key: string }) {
      if (event.key.toLowerCase() == 'w') {
        this.inputData.w_key = 1
      } else if (event.key.toLowerCase() == 'a') {
        this.inputData.a_key = 1
      } else if (event.key.toLowerCase() == 's') {
        this.inputData.s_key = 1
      } else if (event.key.toLowerCase() == 'd') {
        this.inputData.d_key = 1
      }
    },

    // when a key is released, sets input for that key as 0
    keyMonitorUp: function(event: { key: string }) {
      if (event.key.toLowerCase() == 'w') {
        this.inputData.w_key = 0
      } else if (event.key.toLowerCase() == 'a') {
        this.inputData.a_key = 0
      } else if (event.key.toLowerCase() == 's') {
        this.inputData.s_key = 0
      } else if (event.key.toLowerCase() == 'd') {
        this.inputData.d_key = 0
      }
    },

    publish: function() {
      this.sendMessage({
        type: 'mast_gimbal',
        throttles: [
          this.inputData.w_key - this.inputData.s_key,
          this.inputData.d_key - this.inputData.a_key
        ]
      })
    }
  }
}
</script>

<style scoped>
.wrap {
  display: inline-block;
}
</style>
