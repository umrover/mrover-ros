<template>
  <div class="wrapper">
    <div class="box header">
      <img
        src="/static/mrover.png"
        alt="MRover"
        title="MRover"
        width="48"
        height="48"
      />
      <h1>SA Dashboard</h1>
      <div class="spacer"></div>
      <div class="spacer"></div>
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
    <div class="box map light-bg">
      <ERDMap :odom="odom" />
    </div>
    <div class="box waypoints light-bg">
      <ERDWaypointEditor />
    </div>
    <div class="box light-bg cameras">
      <Cameras :primary="true" />
    </div>
    <div>
      <DriveControls />
    </div>
    <div class="box light-bg scoop">
      <EndEffectorUV />
    </div>
    <div class="box light-bg arm">
      <ArmControls />
    </div>
    <div class="box light-bg pdb">
      <PDBFuse />
    </div>
    <div class="box light-bg jointState">
      <JointStateTable :joint-state-data="jointState" :vertical="true" />
    </div>
    <div class="box light-bg moteus">
      <MoteusStateTable :moteus-state-data="moteusState" />
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import ERDMap from "./ERDRoverMap.vue";
import ERDWaypointEditor from "./ERDWaypointEditor.vue";
import DriveControls from "./DriveControls.vue";
import EndEffectorUV from "./EndEffectorUV.vue";
import ArmControls from "./ArmControls.vue";
import PDBFuse from "./PDBFuse.vue";
import * as qte from "quaternion-to-euler";
import Cameras from "./Cameras.vue";
import MoteusStateTable from "./MoteusStateTable.vue";
import JointStateTable from "./JointStateTable.vue";

export default {
  components: {
    ERDMap,
    ERDWaypointEditor,
    DriveControls,
    EndEffectorUV,
    ArmControls,
    PDBFuse,
    Cameras,
    JointStateTable,
    MoteusStateTable,
  },
  data() {
    return {
      // Default coordinates are at NC 53 Parking Lot
      odom: {
        latitude_deg: 42.294864932393835,
        longitude_deg: -83.70781314674628,
        bearing_deg: 0,
      },

      jointState: {},
      // Moteus state table is set up to look for specific keys in moteusState so it can't be empty
      moteusState: {
        name: ["", "", "", "", "", ""],
        error: ["", "", "", "", "", ""],
        state: ["", "", "", "", "", ""],
      },

      // Pubs and Subs
      odom_sub: null,
      tfClient: null,
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
      fixedFrame: "odom",
      // Thresholds to trigger subscription callback
      angularThres: 0.01,
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
      console.log(tf);
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

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
.wrapper {
  display: grid;
  overflow: hidden;
  grid-gap: 10px;
  grid-template-columns: 40vw 5vw 25vw 25vw;
  grid-template-rows: 60px 70vh auto auto auto;
  grid-template-areas:
    "header header header header"
    "map map waypoints waypoints"
    "cameras cameras cameras scoop"
    "arm moteus moteus jointState"
    "pdb moteus moteus jointState";
  font-family: sans-serif;
  height: auto;
}

.box {
  border-radius: 5px;
  padding: 10px;
  border: 1px solid black;
}

.box1 {
  border-radius: 5px;
  background-color: LightGrey;
  padding: 10px;
  border: 1px solid black;
  overflow-y: scroll;
}

.box2 {
  display: block;
}

.light-bg {
  background-color: LightGrey;
}
img {
  border: none;
  border-radius: 0px;
}

.header {
  grid-area: header;
  display: flex;
  align-items: center;
}

.header h1 {
  margin-left: 5px;
}
h2 {
  padding: 2px;
  margin: 0px;
}

.spacer {
  flex-grow: 0.8;
}

.comms {
  display: flex;
  flex-direction: column;
  align-items: flex-start;
}
.comms * {
  margin-top: 2px;
  margin-bottom: 2px;
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

.waypoints {
  grid-area: waypoints;
}

.scoop {
  grid-area: scoop;
}

.arm {
  grid-area: arm;
}

.pdb {
  grid-area: pdb;
}

.jointState {
  grid-area: jointState;
}

.moteus {
  grid-area: moteus;
}

.Joystick {
  font-size: 1em;
  height: 41%;
  width: 93%;
  display: inline-block;
}
.raw-sensors {
  font-size: 1em;
}

.controls {
  font-size: 1em;
  height: 40.5%;
  display: block;
}

ul#vitals li {
  display: inline;
  float: left;
  padding: 0px 10px 0px 0px;
}
</style>
