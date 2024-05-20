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
      mappings: {
        w: 0,
        a: 1,
        s: 2,
        d: 3
      },
      keys: Array(4).fill(0)
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
    // This is necessary instead of just sending in the event listeners
    // The listeners have this strange behavior where they pause for a bit before sending many
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
        type: 'mast_keyboard',
        axes: [],
        buttons: this.keys
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
