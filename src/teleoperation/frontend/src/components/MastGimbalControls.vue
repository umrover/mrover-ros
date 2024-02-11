<template>
  <div class="wrap">
    <div v-show="false" id="key">
      <input @keydown="keyMonitorDown" />
      <input @keyup="keyMonitorUp" />
    </div>
  </div>
</template>

<script lang="ts">
import { mapActions } from 'vuex'
const UPDATE_RATE_S = 1
let interval: number

export default {
  data() {
    return {
      rotation_pwr: 0,
      up_down_pwr: 0,

      keyboard_pub: null,

      inputData: {
        w_key: 0,
        a_key: 0,
        s_key: 0,
        d_key: 0
      }
    }
  },

  beforeUnmount: function () {
    window.clearInterval(interval)
    document.removeEventListener('keyup', this.keyMonitorUp)
    document.removeEventListener('keydown', this.keyMonitorDown)
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
    // When a key is being pressed down, set the power level.
    // Ignore keys that are already pressed to avoid spamming when holding values.
    keyMonitorDown: function (event: { key: string }) {
      if (event.key.toLowerCase() == 'w') {
        if (this.inputData.w_key > 0) {
          return
        }
        this.inputData.w_key = this.up_down_pwr
      } else if (event.key.toLowerCase() == 'a') {
        if (this.inputData.a_key > 0) {
          return
        }
        this.inputData.a_key = this.rotation_pwr
      } else if (event.key.toLowerCase() == 's') {
        if (this.inputData.s_key > 0) {
          return
        }
        this.inputData.s_key = this.up_down_pwr
      } else if (event.key.toLowerCase() == 'd') {
        if (this.inputData.d_key > 0) {
          return
        }
        this.inputData.d_key = this.rotation_pwr
      }

      this.publish()
    },

    // when a key is released, sets input for that key as 0
    keyMonitorUp: function (event: { key: string }) {
      if (event.key.toLowerCase() == 'w') {
        this.inputData.w_key = 0
      } else if (event.key.toLowerCase() == 'a') {
        this.inputData.a_key = 0
      } else if (event.key.toLowerCase() == 's') {
        this.inputData.s_key = 0
      } else if (event.key.toLowerCase() == 'd') {
        this.inputData.d_key = 0
      }

      this.publish()
    },

    publish: function () {
      this.sendMessage({
        type: 'mast_gimbal',
        throttles: [
          this.inputData.d_key - this.inputData.a_key,
          this.inputData.w_key - this.inputData.s_key
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
