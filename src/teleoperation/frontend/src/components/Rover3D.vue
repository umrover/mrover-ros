<template>
  <div class='wrapper'>
    <h3 class='header'>Rover 3D</h3>
    <div id='threejs'></div>
  </div>
</template>

<script lang='ts'>
import { defineComponent } from 'vue'
import { mapState } from 'vuex'
import { threeSetup } from '../rover_three.js'

export default defineComponent({
  data() {
    return {
      threeScene: null,
      temp_positions: ['base', 'a', 'b', 'c', 'd', 'e'],
      positions: [],
    }
  },

  mounted() {
    this.threeScene = threeSetup('threejs')
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == 'fk') {
        msg.position = msg.position.map((x) => isNaN(x) ? 0 : x)
        this.threeScene.fk(msg.position)
      } else if (msg.type == 'ik') {
        this.threeScene.ik(msg.target)
      }
    }
  }
})
</script>

<style scoped>
.wrapper {
  margin: 5px;
}

.header {
  text-align: center;
}

#threejs {
  width: 500px;
  height: 500px;
}
</style>