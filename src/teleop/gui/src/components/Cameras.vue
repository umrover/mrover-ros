<template>
  <div class="wrap">
    <h3>Cameras</h3>
    <div v-if="!bindingsExist" class="input">
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
        :cams="cameras"
        :capacity="parseInt(capacity)"
        @cam_index="setCamIndex($event)"
      />
    </div>
    <h3>All Cameras</h3>
    Capacity:
    <input v-model="capacity" class="rounded" type="Number" min="2" max="4" />
    <button :disabled="!allDisabled()" class="rounded button" @click="showModal = true">
      Setup
    </button>
    <button class="rounded button" @click="resetBindings()">
      Reset
    </button>

    <label for="preset">Presets:</label>
        <select id="preset" v-model="selectedPreset" class="box" required @change="changePreset()">
          <option value="" selected>Select a preset</option>
          <option v-for="option in presets" :value="option">
            {{ option.name }}
          </option>
        </select>

    <div v-if="showModal || deviceID > 8" @close="closeModal()">
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
                @click="closeModal()"
              >
                x
              </button>
            </header>

            <section
              class="modal-body"
              id="modalDescription"
            >
              <slot name="body">
                <p>Which camera is this?</p>
                <p>Device ID: {{ deviceID }}</p>
                <button @click="assignID('NotACamera')">Not a Camera</button>
                <button v-for="name in cameraNames" :key="name" @click="assignID(name)">{{name}}</button>
              </slot>
            </section>

            <footer class="modal-footer">
              <button
                type="button"
                class="btn-green"
                @click="closeModal()"
              >
                Close me!
              </button>
            </footer>
          </div>
        </div>
      </transition>
    </div>


    <div v-for="(c, idx) in cameras" :key="idx" class="camerainfo">
      <CameraInfo
        v-if="c.enabled"
        :id="c.index"
        :name="c.name"
        :stream="getStreamNum(c.index)"
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
      cameras: [],
      cameraIdx: 1,
      cameraName: "",
      capacity: 2,
      streamOrder: [-1, -1, -1, -1],
      //below are variables for bindings
      bindings: null,
      showModal: false,
      cameraNames: [],
      deviceID: 0,
      nameIDMap: new Object(),
      bindingsExist: false,
      //variables for presets
      selectedPreset: "",
      presets: []
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

    this.initCams();

    var names = new ROSLIB.Param({
      ros: this.$ros,
      name: "teleop/camera_list"
    });
    names.get((val) => this.cameraNames = val);

    this.bindings = new ROSLIB.Param({
      ros: this.$ros,
      name: "teleop/camera_bindings"
    });
    this.addBindings();

    var p = new ROSLIB.Param({
      ros: this.$ros,
      name: "teleop/camera_presets"
    });
    p.get((values) => {
      var cams = values.map((v) => v.cams);
      var names = values.map((v) => v.name);
      for(var i = 0; i < cams.length; i++){
        this.presets.push({name: names[i], cams: cams[i]});
      }
    });
  },

  methods: {
    setCamIndex(index) {
      //every time a button is pressed, it changes cam status and adds/removes from stream
      var cam = this.cameras.find((cam) => cam.index == index);
      cam.enabled = !cam.enabled;
      if(cam.enabled) cam.quality = 2; //if enabling camera, turn on medium quality
      this.changeStream(index);
    },

    sendCameras(index) {
      //sends cameras to a service to display on screen
      var res = this.cameras.find((cam) => cam.index == index).quality;
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

    addCameraName() {
      this.cameras.find((cam) => cam.index == this.cameraIdx).name = this.cameraName;
    },

    changeQuality({ index, value }) {
      this.cameras.find((cam) => cam.index == index).quality = value;
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
        this.removeStream(index);
      } else this.addStream(index);
      this.sendCameras(index);
    },

    addStream(index) {
      Vue.set(this.streamOrder, this.streamOrder.indexOf(-1), index);
    },

    removeStream(index) {
      this.streamOrder.splice(this.streamOrder.indexOf(index), 1);
      this.streamOrder.push(-1);
      this.cameras.find((cam) => cam.index == index).quality = -1;  //close the stream when sending it to comms
    },

    getStreamNum(index) {
      return this.streamOrder.indexOf(index);
    },

    initCams() {
      this.cameras = [];
      for(var i = 0; i < 9; i++){
        this.cameras.push(
          {
            name: "Camera " + i,
            enabled: false,
            quality: -1,
            index: i
          }
        );
      }
    },

    allDisabled(){
      var off = true;
      this.cameras.forEach(c => {
        if(c.enabled) off = false;
      });
      return off;
    },

    assignID(name){
      if(name != "NotACamera") this.nameIDMap[name] = this.deviceID;
      this.deviceID += 2;
      if(this.deviceID == 9) this.closeModal(); //iterated through evens and odds, so close
      else if(this.deviceID > 8) this.deviceID = 1;
    },

    addBindings() {
      this.bindings.get((dict) => {
        console.log(dict);
        if(dict) {
          var indices = Object.values(dict);
          var names = Object.keys(dict);
          this.bindingsExist = true;
          if(names.length > 4) this.capacity = 4;
          else this.capacity = names.length;
          var i = this.cameras.length-1;
          while(i >= 0){
            var ind = indices.indexOf(this.cameras[i].index);
            if(ind != -1) { //assign the name of the camera to index if found, else remove camera
              this.cameras[i].name = names[ind];
            }
            else this.cameras.splice(i, 1);
            --i;
          }
        }
      });
    },

    resetBindings() {
      //remove all the cameras from the stream
      this.cameras.forEach((cam) => this.removeStream(cam.index));
      //erase all the bindings
      this.bindings.set(new Object());
      this.bindingsExist = false;
      //reinit cams to default settings
      this.initCams();
      this.selectedPreset = "";
    },

    removeUnused() {
      this.cameras = this.cameras.filter(cam => cam.enabled);
    },

    closeModal() {
      this.showModal = false;
      this.deviceID = 0;
      this.bindings.set(this.nameIDMap);
      this.nameIDMap = new Object();
      this.addBindings();
    },

    changePreset() {
      if(this.selectedPreset){
        this.cameras.forEach((cam) => this.removeStream(cam.index));
        const preset = this.presets.find((name) => this.selectedPreset == name);
        this.cameras.forEach((cam) => {
          const found = preset.cams.includes(cam.name);
          if(found) {
            cam.enabled = true;
            this.changeStream(cam.index);
          }
          else cam.enabled = false;
        });
      }
    }
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
