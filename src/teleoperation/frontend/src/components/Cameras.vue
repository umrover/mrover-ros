<template>
  <div class="wrap row">
    <div class="col">
      <h3>Cameras ({{ num_available }} available)</h3>
      <div class="row justify-content-md-left">
        <div class="form-group col-md-4">
          <label for="Camera Name">Camera name</label>
          <input
            v-model="cameraName"
            type="message"
            class="form-control"
            id="CameraName"
            placeholder="Enter Camera Name"
          />
          <small id="cameraDescrip" class="form-text text-muted"></small>
        </div>
        <div class="form-group col-md-4">
          <label for="Camera ID">Camera ID</label>
          <input
            v-model="cameraIdx"
            type="number"
            min="0"
            max="8"
            class="form-control"
            id="CameraIdx"
            placeholder="Camera ID"
          />
        </div>
        <button class="btn btn-primary custom-btn" @click="addCameraName()">Change Name</button>
      </div>
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
      <div class="info">
        <template v-for="i in camsEnabled.length" :key="i">
          <CameraInfo
            v-if="camsEnabled[i - 1]"
            :id="i - 1"
            :name="names[i - 1]"
            :stream="getStreamNum(i - 1)"
            @newQuality="changeQuality($event)"
            @swapStream="swapStream($event)"
          ></CameraInfo>
        </template>
      </div>
      <div class="d-flex justify-content-end" v-if="isSA">
        <button class="btn btn-primary btn-lg custom-btn" @click="takePanorama()">
          Take Panorama
        </button>
      </div>
    </div>
    <CameraDisplay :streamOrder="streamOrder" :mission="mission"></CameraDisplay>
  </div>
</template>

<script lang="ts">
import CameraSelection from '../components/CameraSelection.vue'
import CameraInfo from '../components/CameraInfo.vue'
import CameraDisplay from './CameraDisplay.vue'
import { mapActions, mapState } from 'vuex'
import { reactive } from 'vue'

export default {
  components: {
    CameraSelection,
    CameraInfo,
    CameraDisplay
  },

  props: {
    primary: {
      type: Boolean,
      required: true
    },
    isSA: {
      type: Boolean,
      required: true
    },
    mission: {
      type: String, // {'sa', 'ik', 'other'}
      required: true
    }
  },
  data() {
    return {
      camsEnabled: reactive(new Array(9).fill(false)),
      names: reactive(Array.from({ length: 9 }, (_, i) => 'Camera: ' + i)),
      cameraIdx: 0,
      cameraName: '',
      capacity: 4,
      qualities: reactive(new Array(9).fill(-1)),
      streamOrder: reactive([]),

      num_available: -1
    }
  },

  watch: {
    message(msg) {
      if (msg.type == 'max_streams') {
        this.streamOrder = new Array(msg.streams).fill(-1)
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
      this.sendMessage({ type: 'max_streams' })
    }, 250)
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    setCamIndex: function (index: number) {
      // console.log(typeof index)
      // console.log(this.camsEnabled[index])
      // every time a button is pressed, it changes cam status and adds/removes from stream
      this.camsEnabled[index] = !this.camsEnabled[index]
      if (this.camsEnabled[index]) this.qualities[index] = 2 //if enabling camera, turn on medium quality
      this.changeStream(index)
    },

    sendCameras: function (index: number) {
      this.sendMessage({
        type: 'sendCameras',
        primary: this.primary,
        device: index,
        resolution: this.qualities[index]
      })
    },

    addCameraName: function () {
      this.names[this.cameraIdx] = this.cameraName
    },

    changeQuality({ index, value }) {
      this.qualities[index] = value
      this.sendCameras(index)
    },

    swapStream({ prev, newest }) {
      var temp = this.streamOrder[prev]
      // Vue.set(this.streamOrder, prev, this.streamOrder[newest]);
      this.streamOrder[prev] = this.streamOrder[newest]
      this.streamOrder[newest] = temp
    },

    changeStream(index: number) {
      const found = this.streamOrder.includes(index)
      if (found) {
        this.streamOrder.splice(this.streamOrder.indexOf(index), 1)
        this.streamOrder.push(-1)
        this.qualities[index] = -1 //close the stream when sending it to comms
      } else this.streamOrder[this.streamOrder.indexOf(-1)] = index
      this.sendCameras(index)
    },

    getStreamNum(index: number) {
      //TODO: check this out
      return this.streamOrder.indexOf(index)
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
