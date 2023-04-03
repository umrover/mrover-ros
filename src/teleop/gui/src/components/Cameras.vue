<template>
  <div class="wrap">
    <h3>Cameras</h3>
    <div class="input">
      Camera name:
      <input v-model="cameraName" class="rounded" type="message" /> Camera
      number:
      <input
        v-model="cameraIdx"
        class="rounded"
        type="Number"
        min="0"
        max="8"
      />
      <button class="rounded button" @click="addCameraName()">
        Change name
      </button>
    </div>
    <div class="cameraselection">
      <CameraSelection
        class="cameraspace1"
        :cams-enabled="camsEnabled"
        :names="names"
        :capacity="parseInt(capacity)"
        @cam_index="setCamIndex($event)"
      />
    </div>
    <h3>All Cameras</h3>
    Capacity:
    <input v-model="capacity" class="rounded" type="Number" min="2" max="4" />
    <button v-if="allDisabled()" class="rounded button" @click="setup()">
      Setup
    </button>

    <div v-if="showModal" @close="showModal = false">
      <transition name="modal-fade">
        <div class="modal-backdrop">
          <div class="modal"
            role="dialog"
          >
            <header
              class="modal-header"
              id="modalTitle"
            >
              <button
                type="button"
                class="btn-close"
                @click="showModal = false"
              >
                x
              </button>
            </header>

            <section
              class="modal-body"
              id="modalDescription"
            >
              <slot name="body">
                Which camera is this?
                <button>Not a Camera</button>
                <button v-for="name in cameraNames" :key="name">{{name}}</button>
              </slot>
            </section>

            <footer class="modal-footer">
              <button
                type="button"
                class="btn-green"
                @click="showModal = false"
              >
                Close me!
              </button>
            </footer>
          </div>
        </div>
      </transition>
    </div>


    <div v-for="i in camsEnabled.length" :key="i" class="camerainfo">
      <CameraInfo
        v-if="camsEnabled[i - 1]"
        :id="i - 1"
        :name="names[i - 1]"
        :stream="getStreamNum(i - 1)"
        @newQuality="changeQuality($event)"
        @swapStream="swapStream($event)"
      ></CameraInfo>
    </div>
  </div>
</template>

<script>
import Vue from "vue";
import ROSLIB from "roslib/src/RosLib";
import CameraSelection from "../components/CameraSelection.vue";
import CameraInfo from "../components/CameraInfo.vue";

export default {
  components: {
    CameraSelection,
    CameraInfo,
  },

  props: {
    primary: {
      type: Boolean,
      required: true,
    },
  },
  data() {
    return {
      camsEnabled: new Array(9).fill(false),
      names: Array.from({ length: 9 }, (_, i) => "Camera: " + i),
      cameraIdx: 1,
      cameraName: "",
      capacity: 2,
      qualities: new Array(9).fill(-1),
      streamOrder: [-1, -1, -1, -1],
      showModal: false,
      cameraNames: []
    };
  },

  watch: {
    capacity: function (newCap, oldCap) {
      if (newCap < oldCap) {
        var numStreaming = this.streamOrder.filter((index) => index != -1);
        var ind = numStreaming.length - 1;
        this.setCamIndex(numStreaming[ind]);
      }
    },
  },

  created: function() {
    var resetService = new ROSLIB.Service({
      ros: this.$ros,
      name: "reset_cameras",
      serviceType: "ResetCameras"
    });
    var request = new ROSLIB.ServiceRequest({
      primary: this.primary,
    });
    resetService.callService(request, (result) => {});

    var names = new ROSLIB.Param({
      ros: this.$ros,
      name: "teleop/camera_list"
    });
    names.get((val) => {
      this.cameraNames = val;
    });
  },

  methods: {
    setCamIndex: function (index) {
      //every time a button is pressed, it changes cam status and adds/removes from stream
      Vue.set(this.camsEnabled, index, !this.camsEnabled[index]);
      if(this.camsEnabled[index]) this.qualities[index] = 2;  //if enabling camera, turn on medium quality
      this.changeStream(index);
    },

    sendCameras: function (index) {
      //sends cameras to a service to display on screen
      var res = this.qualities[index];
      var msg = new ROSLIB.Message({ device: index, resolution: res }); //CameraCmd msg

      var changeCamsService = new ROSLIB.Service({
        ros: this.$ros,
        name: "change_cameras",
        serviceType: "ChangeCameras",
      });

      var request = new ROSLIB.ServiceRequest({
        primary: this.primary,
        camera_cmd: msg,
      });
      changeCamsService.callService(request, (result) => {});
    },

    addCameraName: function () {
      Vue.set(this.names, this.cameraIdx, this.cameraName);
    },

    changeQuality({ index, value }) {
      Vue.set(this.qualities, index, value);
      this.sendCameras(index);
    },

    swapStream({ prev, newest }) {
      var temp = this.streamOrder[prev];
      Vue.set(this.streamOrder, prev, this.streamOrder[newest]);
      Vue.set(this.streamOrder, newest, temp);
    },

    changeStream(index) {
      const found = this.streamOrder.includes(index);
      if (found) {
        this.streamOrder.splice(this.streamOrder.indexOf(index), 1);
        this.streamOrder.push(-1);
        this.qualities[index] = -1;  //close the stream when sending it to comms
      } else Vue.set(this.streamOrder, this.streamOrder.indexOf(-1), index);
      this.sendCameras(index);
    },

    getStreamNum(index) {
      return this.streamOrder.indexOf(index);
    },

    allDisabled(){
      var off = true;
      for(var c in this.camsEnabled){
        if(this.camsEnabled[c]) off = false;
      }
      return off;
    },

    setup(){
      this.showModal = true;

    },
  },
};
</script>

<style scoped>
.rounded {
  border: 1px solid black;
  border-radius: 5px;
}

.button {
  height: 25px;
}

.input > * {
  margin: 5px 0 5px 0;
}

.cameraselection {
  margin: 10px;
}


.modal-backdrop {
    position: fixed;
    top: 0;
    bottom: 0;
    left: 0;
    right: 0;
    background-color: rgba(0, 0, 0, 0.3);
    display: flex;
    justify-content: center;
    align-items: center;
  }

  .modal {
    background: #FFFFFF;
    box-shadow: 2px 2px 20px 1px;
    overflow-x: auto;
    display: flex;
    flex-direction: column;
  }

  .modal-header,
  .modal-footer {
    padding: 15px;
    display: flex;
  }

  .modal-header {
    position: relative;
    border-bottom: 1px solid #eeeeee;
    color: #4AAE9B;
    justify-content: space-between;
  }

  .modal-footer {
    border-top: 1px solid #eeeeee;
    flex-direction: column;
  }

  .modal-body {
    position: relative;
    padding: 20px 10px;
  }

  .btn-close {
    position: absolute;
    top: 0;
    right: 0;
    border: none;
    font-size: 20px;
    padding: 10px;
    cursor: pointer;
    font-weight: bold;
    color: #4AAE9B;
    background: transparent;
  }

  .btn-green {
    color: white;
    background: #4AAE9B;
    border: 1px solid #4AAE9B;
    border-radius: 2px;
  }

  .modal-fade-enter,
  .modal-fade-leave-to {
    opacity: 0;
  }

  .modal-fade-enter-active,
  .modal-fade-leave-active {
    transition: opacity .5s ease;
  }

</style>
