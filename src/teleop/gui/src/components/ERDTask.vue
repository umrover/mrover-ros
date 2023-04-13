<template>
  <div :class="type === 'ES' ? 'wrapper-es' : 'wrapper-edm'">
    <div class="page_header">
      <img src="/static/mrover_logo.png" alt="MRover" title="MRover" width="185" height="53" />
      <h1 v-if="type === 'ES'">ES GUI Dashboard</h1>
      <h1 v-else>EDM GUI Dashboard</h1>
      <div class="help">
        <img
          src="/static/help.png"
          alt="Help"
          title="Help"
          width="48"
          height="48"
        />
      </div>
      <div class="helpscreen"></div>
      <div
        class="helpimages"
        style="
          display: flex;
          align-items: center;
          justify-content: space-evenly;
        "
      >
        <img
          src="/static/joystick.png"
          alt="Joystick"
          title="Joystick Controls"
          style="width: auto; height: 70%; display: inline-block"
        />
      </div>
    </div>
    <div class="box cameras">
      <Cameras :primary="true" />
    </div>
    <div v-if="type === 'EDM'" class="box odom">
      <OdometryReading :odom="odom" />
    </div>
    <div v-if="type === 'EDM'" class="box map">
      <BasicMap :odom="odom" />
    </div>
    <div class="box pdb">
      <PDBFuse />
    </div>
    <div class="box drive-vel-data">
      <JointStateTable :joint-state-data="jointState" :vertical="true" />
    </div>
    <div v-if="type === 'EDM'" class="box waypoint-editor">
      <BasicWaypointEditor />
    </div>
    <div>
      <DriveControls></DriveControls>
    </div>
    <div class="box arm-controls">
      <ArmControls />
    </div>
    <div class="box moteus">
      <MoteusStateTable :moteus-state-data="moteusState" />
    </div>
    <div v-show="false">
      <MastGimbalControls></MastGimbalControls>
    </div>
  </div>
</template>

<script>
import '../assets/style.css';
import { mapGetters } from "vuex";
import * as qte from "quaternion-to-euler";
import ROSLIB from "roslib";

import ArmControls from "./ArmControls.vue";
import Cameras from "./Cameras.vue";
import DriveControls from "./DriveControls.vue";
import MastGimbalControls from "./MastGimbalControls.vue";
import BasicMap from "./BasicRoverMap.vue";
import BasicWaypointEditor from "./BasicWaypointEditor.vue";
import JointStateTable from "./JointStateTable.vue";
import MoteusStateTable from "./MoteusStateTable.vue";
import OdometryReading from "./OdometryReading.vue";
import PDBFuse from "./PDBFuse.vue";

export default {
  components: {
    ArmControls,
    Cameras,
    DriveControls,
    BasicMap,
    BasicWaypointEditor,
    JointStateTable,
    MastGimbalControls,
    MoteusStateTable,
    OdometryReading,
    PDBFuse,
  },

  props: {
    type: {
      type: String,
      required: true,
    },
  },
  data() {
    return {
      // Default coordinates are at NC 53 Parking Lot
      odom: {
        latitude_deg: 42.294864932393835,
        longitude_deg: -83.70781314674628,
        bearing_deg: 0,
        speed: 0,
      },

      // Pubs and Subs
      odom_sub: null,
      tfClient: null,

      // Default object isn't empty, so has to be initialized to ""
      moteusState: {
        name: ["", "", "", "", "", ""],
        error: ["", "", "", "", "", ""],
        state: ["", "", "", "", "", ""],
      },

      jointState: {},
    };
  },

  created: function () {
    this.odom_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/gps/fix",
      messageType: "sensor_msgs/NavSatFix",
    });

    this.tfClient = new ROSLIB.TFClient({
      ros: this.$ros,
      fixedFrame: "map",
      // Thresholds to trigger subscription callback
      angularThres: 0.0001,
      transThres: 0.01,
    });

    // Subscriber for odom to base_link transform
    this.tfClient.subscribe("base_link", (tf) => {
      // Callback for IMU quaternion that describes bearing
      let quaternion = tf.rotation;
      quaternion = [quaternion.w, quaternion.x, quaternion.y, quaternion.z];
      //Quaternion to euler angles
      let euler = qte(quaternion);
      // euler[2] == euler z component
      this.odom.bearing_deg = euler[2] * (180 / Math.PI);
    });

    this.odom_sub.subscribe((msg) => {
      // Callback for latLng to be set
      this.odom.latitude_deg = msg.latitude;
      this.odom.longitude_deg = msg.longitude;
    });

    this.brushless_motors = new ROSLIB.Topic({
      ros: this.$ros,
      name: "drive_status",
      messageType: "mrover/MotorsStatus",
    });

    this.brushless_motors.subscribe((msg) => {
      this.jointState = msg.joint_states;
      this.moteusState = msg.moteus_states;
    });
  },
};
</script>

<style scoped>
.wrapper-edm {
  margin: 0 15px 0 15px;
  display: grid;
  gap: 15px;
  grid-template-columns: auto auto;
  grid-template-rows: auto 20% 10% 20% 40% 40%;
  grid-template-areas:
    "header header"
    "map waypoint-editor"
    "map odom"
    "map arm-controls"
    "cameras drive-vel-data"
    "moteus pdb";
  font-family: sans-serif;
  height: 100%;
}

.wrapper-es {
  margin: 0 15px 0 15px;
  display: grid;
  gap: 15px;
  grid-template-columns: auto auto;
  grid-template-rows: auto 20% 20% 20% 20%;
  grid-template-areas:
    "header header"
    "cameras moteus"
    "cameras moteus"
    "drive-vel-data pdb"
    "arm-controls arm-controls";
  font-family: sans-serif;
  height: 100%;
}

.page_header {
  grid-area: header;
  display: flex;
  align-items: center;
}

.box {
  box-shadow: 2px 2px 8px var(--shadow-color), -2px -2px 8px var(--shadow-color);
}

.helpscreen {
  z-index: 1000000000;
  display: block;
  visibility: hidden;
  background-color: black;
  opacity: 0.8;
  position: absolute;
  left: 0px;
  top: 0px;
  width: 100%;
  height: 100%;
}

.helpimages {
  z-index: 1000000001;
  visibility: hidden;
  position: absolute;
  left: 5%;
  top: 5%;
  width: 90%;
  height: 90%;
}

.help {
  z-index: 1000000002;
  display: flex;
  float: right;
  opacity: 0.8;
  cursor: auto;
}

.help:hover {
  opacity: 1;
  cursor: pointer;
}

.help:hover ~ .helpscreen,
.help:hover ~ .helpimages {
  visibility: visible;
}

.map {
  grid-area: map;
}

.cameras {
  grid-area: cameras;
}

.odom {
  grid-area: odom;
}

.pdb {
  grid-area: pdb;
}

.drive-vel-data {
  grid-area: drive-vel-data;
}

.waypoint-editor {
  grid-area: waypoint-editor;
}

.arm-controls {
  grid-area: arm-controls;
}

.moteus {
  grid-area: moteus;
  min-height: 250px;
}
</style>
