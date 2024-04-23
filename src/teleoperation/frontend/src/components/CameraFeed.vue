<template>
  <div class="wrap">
    <canvas :id="'stream-' + id" v-on:click="handleClick"></canvas>
    <div v-if="mission != 'ZED'">
      <p>{{ name }} • ID: {{ id }}</p>
      <div class="form-group col-md-4">
          <label for="quality">Quality:</label>
          <select
            v-model="quality"
            min="0"
            max="4"
            class="form-control"
            id="quality"
            @change="changeQuality()"
          >
            <option v-for="i in 5" :key="i">{{ i - 1 }}</option>
          </select>
        </div>
      <Checkbox v-if="mission === 'ik'" :name="'IK Camera'" v-on:toggle="toggleIKMode" />
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import { mapActions } from 'vuex'
import Checkbox from './Checkbox.vue'
import CameraSelection from './CameraSelection.vue'

export default defineComponent({
  props: {
    name: {
      type: String,
      required: true
    },
    id: {
      type: Number,
      required: true
    },
    mission: {
      type: String, // {'sa', 'ik', 'auton'}
      required: true
    }
  },
  components: {
    Checkbox,
    CameraSelection
  },

  data() {
    return {
      ws: null as WebSocket | null,
      isUnmounted: false,

      // IK Mode
      IKCam: false,

      quality: 2,

      props: []
    }
  },

  beforeUnmount: function () {
    if (this.ws) this.ws.close()
    this.isUnmounted = true
  },

  mounted: function () {
    this.startStream(this.id)
    // this.$nextTick(() => {
    //   const canvas: HTMLCanvasElement = document.getElementById(
    //     'stream-' + this.id
    //   ) as HTMLCanvasElement
    //   const context = canvas.getContext('2d') ?? new CanvasRenderingContext2D()
    //   context.fillStyle = 'black'
    //   context.fillRect(0, 0, canvas.width, canvas.height)
    // })
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    handleClick: function (event: MouseEvent) {
      if (this.IKCam && this.mission === 'ik') {
        this.sendMessage({ type: 'start_click_ik', data: { x: event.offsetX, y: event.offsetY } })
      }
    },
    changeQuality: function () {
      this.sendMessage({
        type: 'sendCameras',
        primary: this.mission === 'sa',
        device: this.id,
        resolution: parseInt(this.quality)
      })
    },
    
    toggleIKMode: function () {
      this.IKCam = !this.IKCam
    },

    startStream(number: Number) {
      // This function is called as a retry when the websocket closes
      // If our component goes away (unmounts) we should stop trying to reconnect
      // Otherwise it may preempt a new stream that already connected
      if (this.isUnmounted) return

      // Corresponds to H.265
      // I can't figure out what the other values are for... obtained via guess and check
      const STREAM_CODEC = 'hvc1.1.2.L90.90'
      const STREAM_WIDTH = 1280
      const STREAM_HEIGHT = 720
      const STREAM_FPS = 30
      const RECONNECT_TIMEOUT_MS = 3000

      const vertexShaderSource = `
    attribute vec2 xy;

    varying highp vec2 uv;

    void main(void) {
        gl_Position = vec4(xy, 0.0, 1.0);
        // Map vertex coordinates (-1 to +1) to UV coordinates (0 to 1).
        // UV coordinates are Y-flipped relative to vertex coordinates.
        uv = vec2((1.0 + xy.x) / 2.0, (1.0 - xy.y) / 2.0);
    }
    `

      const fragmentShaderSource = `
    varying highp vec2 uv;

    uniform sampler2D texture;

    void main(void) {
        gl_FragColor = texture2D(texture, uv);
    }
    `

      const canvas = document.getElementById(`stream-${number}`) as HTMLCanvasElement
      if (!canvas) return

      // This WebGL stuff seems like a lot, but it's just setting up a shader that can render a texture
      // This texture is uploaded to whenever we get a frame from the decoder
      // More complex than just using a 2D canvas but *may* have lower latency

      const gl = canvas.getContext('webgl2') as WebGL2RenderingContext

      const vertexShader = gl.createShader(gl.VERTEX_SHADER)
      if (!vertexShader) throw new Error('Failed to create vertex shader')

      gl.shaderSource(vertexShader, vertexShaderSource)
      gl.compileShader(vertexShader)
      if (!gl.getShaderParameter(vertexShader, gl.COMPILE_STATUS)) {
        throw gl.getShaderInfoLog(vertexShader)
      }

      const fragmentShader = gl.createShader(gl.FRAGMENT_SHADER)
      if (!fragmentShader) throw new Error('Failed to create fragment shader')

      gl.shaderSource(fragmentShader, fragmentShaderSource)
      gl.compileShader(fragmentShader)
      if (!gl.getShaderParameter(fragmentShader, gl.COMPILE_STATUS)) {
        throw gl.getShaderInfoLog(fragmentShader)
      }

      const shaderProgram = gl.createProgram()
      if (!shaderProgram) throw new Error('Failed to create shader program')

      gl.attachShader(shaderProgram, vertexShader)
      gl.attachShader(shaderProgram, fragmentShader)
      gl.linkProgram(shaderProgram)
      if (!gl.getProgramParameter(shaderProgram, gl.LINK_STATUS)) {
        throw gl.getProgramInfoLog(shaderProgram)
      }
      gl.useProgram(shaderProgram)

      // Vertex coordinates, clockwise from bottom-left
      const vertexBuffer = gl.createBuffer()
      gl.bindBuffer(gl.ARRAY_BUFFER, vertexBuffer)
      gl.bufferData(
        gl.ARRAY_BUFFER,
        new Float32Array([-1.0, -1.0, -1.0, +1.0, +1.0, +1.0, +1.0, -1.0]),
        gl.STATIC_DRAW
      )

      const xyLocation = gl.getAttribLocation(shaderProgram, 'xy')
      gl.vertexAttribPointer(xyLocation, 2, gl.FLOAT, false, 0, 0)
      gl.enableVertexAttribArray(xyLocation)

      const texture = gl.createTexture()
      gl.bindTexture(gl.TEXTURE_2D, texture)
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST)
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST)
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE)
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE)

      const decoder = new VideoDecoder({
        output(frame) {
          canvas.width = frame.displayWidth
          canvas.height = frame.displayHeight
          // Upload the frame to the texture
          gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, frame)
          // Close immediately to free up resources
          // Otherwise the stream will halt
          frame.close()

          gl.viewport(0, 0, canvas.width, canvas.height)
          gl.clearColor(0, 0, 0, 1)
          gl.clear(gl.COLOR_BUFFER_BIT)

          gl.drawArrays(gl.TRIANGLE_FAN, 0, 4)
        },
        error(e) {
          throw e
        }
      })
      // TODO(quintin): Need to know the size ahead of time. Perhaps put in a packet
      decoder.configure({
        codec: STREAM_CODEC,
        codedWidth: STREAM_WIDTH,
        codedHeight: STREAM_HEIGHT
      })

      // TODO(quintin): Set IP too
      this.ws = new WebSocket(`ws://10.1.0.10:808${1 + this.id}`)
      this.ws.binaryType = 'arraybuffer'
      this.ws.onopen = () => {
        console.log(`Connected to server for stream ${number}`)
      }
      this.ws.onclose = () => {
        console.log(`Disconnected from server for stream ${number}`)
        decoder.close()
        // This recursive-ness stops after the canvas element is removed
        setTimeout(() => this.startStream(number), RECONNECT_TIMEOUT_MS)
      }
      this.ws.onerror = () => {
        if (this.ws) this.ws.close()
      }
      this.ws.onmessage = (event) => {
        // TODO(quintin): Should the values besides "data" be set better? Parsed from the packet?
        decoder.decode(
          new EncodedVideoChunk({
            type: 'key',
            timestamp: performance.now(),
            duration: 1000 / STREAM_FPS,
            data: event.data
          })
        )
      }
    }
  }
})
</script>
<style>
.wrap {
  width: fit-content;
  display: flex;
  flex-direction: column;
  gap: 5px;
}

canvas {
  width:640px;
  height:480px;
}
</style>

