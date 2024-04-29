<template>
  <div class="wrap row">
    <div class="col">
      <div class="cameraselection"> 
        <CameraSelection
          :cams-enabled="camsEnabled"
          :names="names"
          :capacity="capacity"
          @cam_index="setCamIndex($event)"
        />
      </div>
    </div>
    <div class="col">
      <h3>All Cameras</h3>
      <div class="d-flex justify-content-end" v-if="isSA">
        <button class="btn btn-primary btn-lg" @click="takePanorama()">
          Take Panorama
        </button>
      </div>
    </div>
    <CameraDisplay :streamOrder="streamOrder" :mission="mission" :names="names" :ports="ports" :qualities="qualities"></CameraDisplay>
  </div>
</template>

<script lang="ts">
import CameraSelection from '../components/CameraSelection.vue'
import CameraDisplay from './CameraDisplay.vue'
import { mapActions, mapState } from 'vuex'
import { reactive } from 'vue'

export default {
  components: {
    CameraSelection,
    CameraDisplay
  },

  props: {
    isSA: {
      type: Boolean,
      required: true
    },
    mission: {
      type: String, // {'sa', 'ik', 'other'}
      required: true
    },
  },
  data() {
    return {
      camsEnabled: reactive(new Array(9).fill(false)),
      names: reactive([]),
      ports: reactive([]),
      cameraIdx: 0,
      cameraName: '',
      capacity: 4,
      streamOrder: reactive([]),
    }
  },

  watch: {
    message(msg) {
      if (msg.type == 'max_streams') {
        this.streamOrder = new Array(msg.streams).fill(-1)
      }
      if (msg.type == 'camera_info') {
        this.names = msg.names
        this.ports = msg.ports
      }
    },
    capacity: function (newCap, oldCap) {
      if (newCap < oldCap) {
        var numStreaming = this.streamOrder.filter((index) => index != -1)
        var ind = numStreaming.length - 1
        this.setCamIndex(numStreaming[ind])
      }
    }
  },

  computed: {
    ...mapState('websocket', ['message']),
    color: function () {
      return this.camsEnabled ? 'btn-success' : 'btn-secondary'
    }
  },

  created: function () {
    window.setTimeout(() => {
      this.sendMessage({ type: 'max_streams' }),
      this.sendMessage({ type: 'camera_info' })
    }, 250)
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    setCamIndex: function (index: number) {
      // every time a button is pressed, it changes cam status and adds/removes from stream
      this.camsEnabled[index] = !this.camsEnabled[index]
      this.changeStream(index)
    },

    addCameraName: function () {
      this.names[this.cameraIdx] = this.cameraName
    },

    changeStream(index: number) {
      const found = this.streamOrder.includes(index)
      if (found) {
        this.streamOrder.splice(this.streamOrder.indexOf(index), 1)
        this.streamOrder.push(-1)
      } else this.streamOrder[this.streamOrder.indexOf(-1)] = index
    },

    takePanorama() {
      this.sendMessage({ type: 'takePanorama' })
    }
  }
}
</script>

<style scoped>
.cameraselection {
  margin: 10px;
}

.custom-btn {
  width: 150px;
  height: 50px;
}

.info {
  height: 200px;
  overflow-y: auto;
}
</style>
