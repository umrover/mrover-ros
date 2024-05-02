<template>
  <div class="wrap">
    <canvas :id="'stream-' + id" v-on:click="handleClick"></canvas>
    <div v-if="mission != 'ZED'">
      <p>{{ name }} • ID: {{ id }}</p>
      <Checkbox v-if="mission === 'ik'" :name="'IK Camera'" v-on:toggle="toggleIKMode" />
    </div>
  </div>
</template>
<script lang="ts">
import {defineComponent} from 'vue'
import {mapActions} from 'vuex'
import Checkbox from './Checkbox.vue'

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
      type: String, // // {'ish', 'ik', 'sa', 'auton', 'ZED'}
      required: true
    }
  },
  components: {
    Checkbox
  },

  data() {
    return {
      ws: null as WebSocket | null,
      isUnmounted: false,

      // IK Mode
      IKCam: false,
    }
  },

  beforeUnmount: function () {
    if (this.ws) this.ws.close()
    this.isUnmounted = true
  },

  mounted: function () {
    this.startStream(this.id)
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    handleClick: function (event: MouseEvent) {
      if (this.IKCam && this.mission === 'ik') {
        this.sendMessage({type: 'start_click_ik', data: {x: event.offsetX, y: event.offsetY}})
      }
    },
    
    toggleIKMode: function () {
      this.IKCam = !this.IKCam
    },

    startStream(number: Number, attempt: Number = 0) {
      // This function is called as a retry when the websocket closes
      // If our component goes away (unmounts) we should stop trying to reconnect
      // Otherwise it may preempt a new stream that already connected
      if (this.isUnmounted) return

      // Corresponds to H.265
      // I can't figure out what the other values are for... obtained via guess and check
      const STREAM_CODEC_MAP = {
        0: 'hvc1.1.2.L90.90',
        1: 'avc1.64001f',
      }
      const STREAM_RESOLUTION_MAP = {
        0: [640, 480],
        1: [1280, 720],
        2: [1920, 1080],
      }
      const STREAM_FPS = 30
      const RECONNECT_TIMEOUT_MS = 1000

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
      let isDecoderConfigured = false

      // TODO(quintin): Set IP too
      const ip = attempt % 2 === 0 ? '10.1.0.10' : 'localhost'
      console.log(`Attempting to connect to server for stream ${number} at ${ip}...`)
      this.ws = new WebSocket(`ws://${ip}:808${1 + this.id}`)
      const timeoutId = setTimeout(() => {
        if (this.ws && this.ws.readyState !== WebSocket.OPEN) {
          console.log(`Timed out waiting for stream ${number}`)
          this.ws.close()
        }
      }, 2000)
      this.ws.binaryType = 'arraybuffer'
      this.ws.onopen = () => {
        console.log(`Opened socket for stream ${number}`)
        clearTimeout(timeoutId)
      }
      this.ws.onclose = () => {
        console.log(`Closed socket for stream ${number}`)
        decoder.close()
        // This recursive-ness stops after the canvas element is removed
        setTimeout(() => this.startStream(number, attempt + 1), RECONNECT_TIMEOUT_MS * (attempt + 1))
      }
      this.ws.onerror = () => {
        if (this.ws) this.ws.close()
      }
      this.ws.onmessage = (event) => {
        // TODO(quintin): Should the values besides "data" be set better? Parsed from the packet?
        const [resolutionId, codecId] = new Uint8Array(event.data.slice(0, 2))
        const [streamWidth, streamHeight] = STREAM_RESOLUTION_MAP[resolutionId]
        const codec = STREAM_CODEC_MAP[codecId]
        if (!isDecoderConfigured) {
          isDecoderConfigured = true
          decoder.configure({
            codec: codec,
            width: streamWidth,
            height: streamHeight
          })
        }
        decoder.decode(
            new EncodedVideoChunk({
              type: 'key',
              timestamp: performance.now(),
              duration: 1000 / STREAM_FPS,
              data: event.data.slice(2, -1)
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
  width: 640px;
  height: 480px;
}
</style>

