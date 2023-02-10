<template>
    <div class="wrap">
      <div class="block box1">
      <h3>Cameras</h3>
      <div class="input">
        Camera name: <input class="box" type='message' v-model ='cameraName'>
        <br>Camera number: <input class="box" type='Number' min="0" max="8" v-model ='cameraIdx'>
        <button class="button" v-on:click="addCameraName()">Change name</button>
      </div>
      <div class="cameraselection">
        <CameraSelection v-bind:camsEnabled="camsEnabled" v-bind:names="names" v-bind:capacity="parseInt(capacity)" v-on:cam_index="setCamIndex($event)"/>
      </div>
      </div>
      <div class="block box2">
      <h3>All Cameras</h3>
      Capacity: <input class="box" type='Number' min="2" max="4" v-model ='capacity'>
      <div class="camerainfo" v-for="i in camsEnabled.length" :key="i">
        <CameraInfo v-if="camsEnabled[i-1]" v-bind:name="names[i-1]" v-bind:id="i-1" v-on:newQuality="changeQuality($event)" v-on:swapStream="swapStream($event)" v-bind:stream="getStreamNum(i-1)"></CameraInfo>
      </div>
      </div>
    </div>
</template>

<script>
import Vue from "vue";
import ROSLIB from "roslib/src/RosLib";
import CameraSelection from "../components/CameraSelection.vue";
import CameraInfo from "../components/CameraInfo.vue";
import '../assets/style.css';

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
      qualities: new Array(9).fill(1),
      streamOrder: [-1, -1, -1, -1],
    };
  },

  watch: {
    capacity: function (newCap, oldCap) {
      if (newCap < oldCap) {
        var numStreaming = this.streamOrder.filter((index) => index != -1);
        var ind = numStreaming.length - 1;
        Vue.set(
          this.camsEnabled,
          numStreaming[ind],
          !this.camsEnabled[numStreaming[ind]]
        );
        this.changeStream(numStreaming[ind]);
      }
    },
  },

  methods: {
    setCamIndex: function (index) {
      //every time a button is pressed, it changes cam status and adds/removes from stream
      Vue.set(this.camsEnabled, index, !this.camsEnabled[index]);
      this.changeStream(index);
      this.sendCameras();
    },

    sendCameras: function () {
      //sends cameras to a service to display on screen
      var msgs = [];
      for (var i = 0; i < 4; i++) {
        var camId = this.streamOrder[i];
        var res = this.qualities[camId];
        msgs.push(new ROSLIB.Message({ device: camId, resolution: res })); //CameraCmd msg
      }

      var changeCamsService = new ROSLIB.Service({
        ros: this.$ros,
        name: "change_cameras",
        serviceType: "ChangeCameras",
      });

      var request = new ROSLIB.ServiceRequest({
        primary: this.primary,
        camera_cmds: msgs,
      });
      changeCamsService.callService(request, (result) => {});
    },

    addCameraName: function () {
      Vue.set(this.names, this.cameraIdx, this.cameraName);
    },

    changeQuality({ index, value }) {
      Vue.set(this.qualities, index, value);
      this.sendCameras();
    },

    swapStream({ prev, newest }) {
      var temp = this.streamOrder[prev];
      Vue.set(this.streamOrder, prev, this.streamOrder[newest]);
      Vue.set(this.streamOrder, newest, temp);
      this.sendCameras();
    },

    changeStream(index) {
      const found = this.streamOrder.includes(index);
      if (found) {
        this.streamOrder.splice(this.streamOrder.indexOf(index), 1);
        this.streamOrder.push(-1);
      } else Vue.set(this.streamOrder, this.streamOrder.indexOf(-1), index);
    },

    getStreamNum(index) {
      return this.streamOrder.indexOf(index);
    },
  },
};
</script>

<style scoped>

.wrap {
  width: 100%;
  display: flex;
}

.input > * {
  margin: 5px 0 5px 0;
}

.button {
  margin: 2px;
}

.block {
  display: block;
}

.box1 {
  width: fit-content;
  margin: 10px;
}

.box2 {
  width:100%;
  margin: 10px;
  overflow-y: scroll;
}

.cameraselection {
  margin: 10px;
}
</style>
