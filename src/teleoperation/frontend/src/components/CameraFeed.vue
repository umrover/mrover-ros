<template>
  <div class="wrap">
    <canvas :id="'canvas' + id" v-on:click="handleClick"></canvas>
    <Checkbox :name="'IK Camera'" v-on:toggle="toggleIKMode" />
  </div>
</template>
<!-- <script src="../../../streaming/embuild/stream_client.js"></script> -->

<script lang="ts">
import { defineComponent } from 'vue'
import Checkbox from './Checkbox.vue'
import { mapActions } from 'vuex'
import '/streaming/stream_client.js?url'

export default defineComponent({
  props: {
    id: {
      type: Number,
      required: true
    },
    port: {
      type: Number,
      required: true
    }
  },
  components: {
    Checkbox
  },
  data() {
    return {
      IKCam: false
    }
  },
  mounted: function () {
    this.$nextTick(() => {
      const canvas: HTMLCanvasElement = document.getElementById(
        'canvas' + this.id
      ) as HTMLCanvasElement
      const context = canvas.getContext('2d') ?? new CanvasRenderingContext2D()
      context.fillStyle = 'black'
      context.fillRect(0, 0, canvas.width, canvas.height)
    })
  },
  methods: {
    ...mapActions('websocket', ['sendMessage']),
    handleClick: function (event: MouseEvent) {
      if (this.IKCam) {
        this.sendMessage({type: 'start_click_ik', data: {x: event.offsetX, y: event.offsetY}})
      }
    },
    toggleIKMode: function () {
      this.IKCam = !this.IKCam
    }
  }
})
</script>

<style scoped>
.wrap {
  width: fit-content;
}
canvas {
  width: 640px;
  height: 480px;
}
</style>
