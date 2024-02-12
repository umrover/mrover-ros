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
      <MCUReset class="mcu_reset"></MCUReset>
      <div class="spacer"></div>
      <CommReadout class="comms"></CommReadout>
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
      <BasicMap :odom="odom" />
    </div>
    <div class="box waypoints light-bg">
      <BasicWaypointEditor :odom="odom" />
    </div>
    <div class="box light-bg cameras">
      <Cameras :primary="true" />
    </div>
    <div>
      <DriveControls />
    </div>
    <div class="box light-bg arm">
      <SAArmControls />
    </div>
    <div class="box light-bg pdb">
      <PDBFuse />
    </div>
    <div class="box light-bg jointState">
      <JointStateTable :joint-state-data="jointState" :vertical="true" />
    </div>
    <div class="box light-bg moteus">
      <DriveMoteusStateTable :moteus-state-data="moteusState" />
    </div>
    <div class="box light-bg limit">
      <h3>Limit Switches</h3>
      <LimitSwitch :switch_name="'sa_joint_1'" :name="'Joint 1 Switch'" />
      <LimitSwitch :switch_name="'sa_joint_2'" :name="'Joint 2 Switch'" />
      <LimitSwitch :switch_name="'sa_joint_3'" :name="'Joint 3 Switch'" />
      <LimitSwitch :switch_name="'sampler'" :name="'Sampler Switch'" />
    </div>
    <div class="box light-bg calibration">
      <h3>Calibrations</h3>
      <div class="calibration-checkboxes">
        <CalibrationCheckbox
          :name="'Joint 1 Calibration'"
          :joint_name="'sa_joint_1'"
          :calibrate_topic="'sa_is_calibrated'"
        />
        <CalibrationCheckbox
          :name="'Joint 2 Calibration'"
          :joint_name="'sa_joint_2'"
          :calibrate_topic="'sa_is_calibrated'"
        />
        <CalibrationCheckbox
          :name="'Joint 3 Calibration'"
          :joint_name="'sa_joint_3'"
          :calibrate_topic="'sa_is_calibrated'"
        />
      </div>
      <MotorAdjust
        :options="[
          { name: 'sa_joint_1', option: 'Joint 1' },
          { name: 'sa_joint_2', option: 'Joint 2' },
          { name: 'sa_joint_3', option: 'Joint 3' }
        ]"
      />
    </div>
    <div class="box light-bg carousel">
      <Carousel></Carousel>
    </div>
    <div v-show="false">
      <MastGimbalControls></MastGimbalControls>
    </div>
    <div class="box light-bg odom">
      <OdometryReading :odom="odom"></OdometryReading>
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import BasicMap from "./BasicRoverMap.vue";
import BasicWaypointEditor from "./BasicWaypointEditor.vue";
import DriveControls from "./DriveControls.vue";
import MastGimbalControls from "./MastGimbalControls.vue";
import SAArmControls from "./SAArmControls.vue";
import PDBFuse from "./PDBFuse.vue";
import Cameras from "./Cameras.vue";
import DriveMoteusStateTable from "./DriveMoteusStateTable.vue";
import JointStateTable from "./JointStateTable.vue";
import LimitSwitch from "./LimitSwitch.vue";
import CalibrationCheckbox from "./CalibrationCheckbox.vue";
import CommReadout from "./CommReadout.vue";
import MCUReset from "./MCUReset.vue";
import MotorAdjust from "./MotorAdjust.vue";
import OdometryReading from "./OdometryReading.vue";
import Carousel from "./Carousel.vue";
import { disableAutonLED, quaternionToMapAngle } from "../utils.js";

export default {
  components: {
    BasicMap,
    BasicWaypointEditor,
    Cameras,
    DriveControls,
    JointStateTable,
    MastGimbalControls,
    DriveMoteusStateTable,
    PDBFuse,
    SAArmControls,
    LimitSwitch,
    CalibrationCheckbox,
    CommReadout,
    MCUReset,
    MotorAdjust,
    OdometryReading,
    Carousel
  },
  data() {
    return {
      // Default coordinates are at NC 53 Parking Lot
      odom: {
        latitude_deg: 42.294864932393835,
        longitude_deg: -83.70781314674628,
        bearing_deg: 0
      },

      brushless_motors_sub: null,

      jointState: {},
      // Moteus state table is set up to look for specific keys in moteusState so it can't be empty
      moteusState: {
        name: ["", "", "", "", "", ""],
        error: ["", "", "", "", "", ""],
        state: ["", "", "", "", "", ""]
      },

      // Pubs and Subs
      odom_sub: null,
      tfClient: null
    };
  },

  created: function () {
    disableAutonLED(this.$ros);

    this.odom_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/gps/fix",
      messageType: "sensor_msgs/NavSatFix"
    });

    this.odom_sub.subscribe((msg) => {
      // Callback for latLng to be set
      this.odom.latitude_deg = msg.latitude;
      this.odom.longitude_deg = msg.longitude;
    });

    this.tfClient = new ROSLIB.TFClient({
      ros: this.$ros,
      fixedFrame: "odom",
      // Thresholds to trigger subscription callback
      angularThres: 0.0001,
      transThres: 0.01
    });

    // Subscriber for odom to base_link transform
    this.tfClient.subscribe("base_link", (tf) => {
      // Callback for IMU quaternion that describes bearing
      this.odom.bearing_deg = quaternionToMapAngle(tf.rotation);
    });

    this.brushless_motors_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "drive_status",
      messageType: "mrover/MotorsStatus"
    });

    this.brushless_motors_sub.subscribe((msg) => {
      this.jointState = msg.joint_states;
      this.moteusState = msg.moteus_states;
    });
  }
};
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: auto auto auto;
  grid-template-rows: 60px 50vh auto auto auto;
  grid-template-areas:
    "header header header header"
    "map map waypoints waypoints"
    "odom cameras cameras cameras"
    "arm limit calibration carousel"
    "pdb moteus jointState jointState";
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
  margin-right: 5px;
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

.limit {
  grid-area: limit;
}

.odom {
  grid-area: odom;
}

.calibration {
  grid-area: calibration;
  display: flex;
  flex-direction: column;
}

.carousel {
  grid-area: carousel;
}

.calibration-checkboxes {
  margin: -4% 0 1% 0;
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
