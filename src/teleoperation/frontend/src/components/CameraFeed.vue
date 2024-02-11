<template>
  <div class="wrap">
    <h3 class="header">H.265 Camera Stream</h3>
    <canvas id="canvas"></canvas>
  </div>
</template>
<!-- <script src="../../../streaming/embuild/stream_client.js"></script> -->

<script lang="ts">
import { defineComponent, nextTick } from 'vue'
import '/streaming/stream_client.js?url'

export default defineComponent({
  data() {
    return {}
  },
  mounted: function () {
    this.$nextTick(() => {
      const canvas: HTMLCanvasElement = document.getElementById('canvas') as HTMLCanvasElement
      const context = canvas.getContext('2d') ?? new CanvasRenderingContext2D()
      context.fillStyle = 'black'
      context.fillRect(0, 0, canvas.width, canvas.height)
    })
  }
})
</script>

<style scoped>
.wrap {
  width: fit-content;
  margin: 5px;
  border: 1px solid black;
}

.header {
  text-align: center;
}
</style>
