<template>
  <div class="wrap">
    <canvas :id="'stream-' + id" v-on:click="handleClick"></canvas>
    <label for="site">Site on Camera:</label>
    <select name="site" id="site" v-model="site">
      <option value="A">A</option>
      <option value="B">B</option>
      <option value="C">C</option>
    </select>
    <button v-on:click="downloadScreenshot">Capture Screenshot</button>
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
      IKCam: false,
      site: 'A'
    }
  },
  mounted: function () {
    this.$nextTick(() => {
      const canvas: HTMLCanvasElement = document.getElementById(
        'stream-' + this.id
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
        this.sendMessage({ type: 'start_click_ik', data: { x: event.offsetX, y: event.offsetY } })
      }
    },
    toggleIKMode: function () {
      this.IKCam = !this.IKCam
    },
    downloadScreenshot: function () {
      const currentdate = new Date()
      const dateString =
        currentdate.getMonth() +
        1 +
        '-' +
        currentdate.getDate() +
        '-' +
        currentdate.getFullYear() +
        ' @ ' +
        currentdate.getHours() +
        ':' +
        currentdate.getMinutes() +
        ':' +
        currentdate.getSeconds()
      var link = document.createElement('a')
      console.log('dateString', dateString)
      link.download = 'Site_' + this.site + ' ' + dateString + '.png'
      let canvas = document.getElementById('stream-' + this.id) as HTMLCanvasElement
      link.href = canvas.toDataURL()
      link.click()
      link.remove()
    }
  }
})
</script>

<style scoped>
.wrap {
  width: fit-content;
  display: flex;
  flex-direction: column;
  gap: 5px;
}
</style>
