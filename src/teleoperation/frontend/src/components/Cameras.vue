<template>
  <div class="wrap row">
    <div class="col">
      <div class="row justify-content-md-left">
        <div class="form-group col-md-4">
          <label for="Camera Name">Camera name</label>
          <input
            v-model="cameraName"
            type="text"
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
      <div class="row text-center align-items-center">
        <div class="col">
          <h3>All Cameras</h3>
        </div>
          <div class="col" v-if="mission === 'ish'">
            <button class="btn btn-primary" @click="takePanorama()">
              Take Panorama
            </button>
          </div>
          <div class="col" v-if="mission === 'ish'">
            <p class="my-auto percent">{{ (percent*100).toFixed(2) }}%</p>
          </div>
      </div>
    </div>
    <CameraDisplay :streamOrder="streamOrder" :mission="mission" :names="names" />
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
    mission: {
      type: String, // {'ish', 'ik', 'sa', 'auton'}
      required: true
    }
  },
  data() {
    return {
      camsEnabled: reactive(new Array(9).fill(false)),
      names: reactive(Array.from({ length: 9 }, (_, i) => 'Camera: ' + i)),
      cameraIdx: 0,
      cameraName: '',
      capacity: 9,
      streamOrder: [-1, -1, -1, -1, -1, -1, -1, -1, -1],
      percent: 0
    }
  },

  watch: {
    message(msg) {
      if (msg.type == 'pano_feedback') {
        this.percent = msg.percent;
      }
    },
    capacity: function (newCap, oldCap) {
      if (newCap < oldCap) {
        const numStreaming = this.streamOrder.filter(index => index != -1)
        const index = numStreaming.length - 1
        this.setCamIndex(numStreaming[index])
      }
    }
  },

  computed: {
    ...mapState('websocket', ['message']),
    color: function() {
      return this.camsEnabled ? 'btn-success' : 'btn-secondary'
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    setCamIndex: function(index: number) {
      // every time a button is pressed, it changes cam status and adds/removes from stream
      this.camsEnabled[index] = !this.camsEnabled[index]
      this.changeStream(index)
    },

    addCameraName: function() {
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

.percent {
  font-size: large;
}
</style>