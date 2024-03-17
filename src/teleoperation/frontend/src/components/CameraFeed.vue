<template>
  <div class="wrap">
    <canvas :id="'canvas'+id" v-on:click="handleClick"></canvas>
  </div>
</template>
<!-- <script src="../../../streaming/embuild/stream_client.js"></script> -->

<script lang="ts">
import { defineComponent } from 'vue'
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
  data() {
    return {
    }
  },
  mounted: function () {
    this.$nextTick(() => {
      const canvas: HTMLCanvasElement = document.getElementById('canvas'+this.id) as HTMLCanvasElement
      const context = canvas.getContext('2d') ?? new CanvasRenderingContext2D()
      context.fillStyle = 'black'
      context.fillRect(0, 0, canvas.width, canvas.height)
    })
  },
  methods:
  {
    handleClick: function(event: MouseEvent) {
      console.log('offsetX', event.offsetX)
      console.log('offsetY', event.offsetY)
    }
  }
})
</script>

<style scoped>
.wrap {
  width: fit-content;
}
</style>
