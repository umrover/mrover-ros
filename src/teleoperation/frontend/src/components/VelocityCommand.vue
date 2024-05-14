<template>
  <div>
    Velocity Command<br />
    Lin: {{ linear_x.toFixed(3) }} m/s<br />
    Ang: {{ angular_z.toFixed(3) }} rad/s
  </div>
</template>

<script lang="ts">
import { mapState } from 'vuex'
export default {
  data() {
    return {
      linear_x: 0,
      angular_z: 0
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if(msg.type == 'cmd_vel') {
        this.linear_x = msg.linear.x;
        this.angular_z = msg.angular.z;
      }
    }
  }
}
</script>

<style scoped></style>
