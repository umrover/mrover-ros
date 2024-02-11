<template>
  <div class="wrap">
    <h3>Cameras ({{ num_available }} available)</h3>
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
    Write Frames to File:
    <ToggleButton
      id="frames_to_file"
      :current-state="toggle_enabled"
      label-enable-text="On"
      label-disable-text="Off"
      @change="toggleLimitSwitch"
    />
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
import { mapGetters, mapMutations } from "vuex";
import ToggleButton from "./ToggleButton.vue";

export default {
  components: {
    CameraSelection,
    CameraInfo,
    ToggleButton
},

  props: {
    primary: {
      type: Boolean,
      required: true
    }
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
      toggle_enabled: false,

      available_sub: null,
      num_available: -1
    };
  },

  // computed: {
  //   ...mapGetters("cameras", {
  //     camsEnabled: "camsEnabled",
  //     names: "names",
  //     cameraIdx: "cameraIdx",
  //     cameraName: "cameraName",
  //     capacity: "capacity",
  //     qualities: "qualities",
  //     streamOrder: "streamOrder"
  //   })
  // },

  watch: {
    capacity: function (newCap, oldCap) {
      if (newCap < oldCap) {
        var numStreaming = this.streamOrder.filter((index) => index != -1);
        var ind = numStreaming.length - 1;
        this.setCamIndex(numStreaming[ind]);
      }
    }
  },

  created: function () {
    var resetService = new ROSLIB.Service({
      ros: this.$ros,
      name: "reset_cameras",
      serviceType: "ResetCameras"
    });
    var request = new ROSLIB.ServiceRequest({
      primary: this.primary
    });
    resetService.callService(request, (result) => {});

    this.available_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "available_cameras",
      messageType: "mrover/AvailableCameras"
    });

    this.available_sub.subscribe((msg) => {
      this.num_available = msg.num_available;
    });
  },

  methods: {
    setCamIndex: function (index) {
      //every time a button is pressed, it changes cam status and adds/removes from stream
      Vue.set(this.camsEnabled, index, !this.camsEnabled[index]);
      if (this.camsEnabled[index]) this.qualities[index] = 2; //if enabling camera, turn on medium quality
      this.changeStream(index);
    },

    sendCameras: function (index) {
      //sends cameras to a service to display on screen
      var res = this.qualities[index];
      var msg = new ROSLIB.Message({ device: index, resolution: res, write_frames_to_file: this.toggle_enabled}); //CameraCmd msg

      var changeCamsService = new ROSLIB.Service({
        ros: this.$ros,
        name: "change_cameras",
        serviceType: "ChangeCameras"
      });

      var request = new ROSLIB.ServiceRequest({
        primary: this.primary,
        camera_cmd: msg
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
        this.qualities[index] = -1; //close the stream when sending it to comms
      } else Vue.set(this.streamOrder, this.streamOrder.indexOf(-1), index);
      this.sendCameras(index);
    },

    //
    getStreamNum(index) {
      return this.streamOrder.indexOf(index);
    },

    toggleLimitSwitch() {
      this.toggle_enabled = !this.toggle_enabled;
    },

    // ...mapMutations("cameras", {
    //   setCamsEnabled: "setCamsEnabled",
    //   setNames: "setNames",
    //   setCameraIdx: "setCameraIdx",
    //   setCameraName: "setCameraName",
    //   setCapacity: "setCapacity",
    //   setQualities: "setQualities",
    //   setStreamOrder: "setStreamOrder"
    // }),
  }
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
</style>
