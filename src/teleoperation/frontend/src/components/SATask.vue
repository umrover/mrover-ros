<template>
  <div class="wrapper">
    <div class="shadow p-3 mb-5 header">
      <h1>SA Dashboard</h1>
      <!-- <MCUReset class="mcu_reset"></MCUReset>
        <CommReadout class="comms"></CommReadout> -->
      <img class="logo" src="/mrover.png" alt="MRover" title="MRover" width="200" />
      <div class="help">
        <img src="/help.png" alt="Help" title="Help" width="48" height="48" />
      </div>
      <div class="helpscreen"></div>
      <div
        class="helpimages"
        style="display: flex; align-items: center; justify-content: space-evenly"
      >
        <img
          src="/joystick.png"
          alt="Joystick"
          title="Joystick Controls"
          style="width: auto; height: 70%; display: inline-block"
        />
      </div>
    </div>
    <div class="shadow p-3 rounded map">
      <BasicMap :odom="odom" />
    </div>
    <div class="shadow p-3 rounded waypoints">
      <BasicWaypointEditor :odom="odom" />
    </div>
    <div class="shadow p-3 rounded cameras">
      <Cameras :primary="true" />
    </div>
    <div>
      <DriveControls />
    </div>
    <!-- <div class="shadow p-3 rounded arm">
        <SAArmControls />
      </div> -->
    <div class="shadow p-3 rounded pdb">
      <PDBFuse />
    </div>
    <div class="shadow p-3 rounded jointState">
      <MotorsStatusTable :motorData="motorData" :vertical="true" />
    </div>
    <div class="shadow p-3 rounded moteus">
      <DriveMoteusStateTable :moteus-state-data="moteusState" />
    </div>
    <div class="shadow p-3 rounded limit">
      <h3>Limit Switches</h3>
      <LimitSwitch :switch_name="'sa_joint_1'" :name="'Joint 1 Switch'" />
      <LimitSwitch :switch_name="'sa_joint_2'" :name="'Joint 2 Switch'" />
      <LimitSwitch :switch_name="'sa_joint_3'" :name="'Joint 3 Switch'" />
      <LimitSwitch :switch_name="'scoop'" :name="'Scoop Switch'" />
    </div>
    <div class="shadow p-3 rounded calibration">
      <h3 style="margin-bottom: 25px">Calibrations</h3>
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
    <div v-show="false">
      <MastGimbalControls></MastGimbalControls>
    </div>
    <div class="shadow p-3 rounded odom">
      <OdometryReading :odom="odom"></OdometryReading>
    </div>
  </div>
</template>

<script lang="ts">
import BasicMap from './BasicRoverMap.vue'
import BasicWaypointEditor from './BasicWaypointEditor.vue'
import DriveControls from './DriveControls.vue'
import MastGimbalControls from './MastGimbalControls.vue'
//   import SAArmControls from "./SAArmControls.vue";
import PDBFuse from './PDBFuse.vue'
import Cameras from './Cameras.vue'
import DriveMoteusStateTable from './DriveMoteusStateTable.vue'
import MotorsStatusTable from './MotorsStatusTable.vue'
import LimitSwitch from './LimitSwitch.vue'
import CalibrationCheckbox from './CalibrationCheckbox.vue'
//   import CommReadout from "./CommReadout.vue";
//   import MCUReset from "./MCUReset.vue";
import MotorAdjust from './MotorAdjust.vue'
import OdometryReading from './OdometryReading.vue'
import { disableAutonLED, quaternionToMapAngle } from '../utils.js'

export default {
  components: {
    BasicMap,
    BasicWaypointEditor,
    Cameras,
    DriveControls,
    MotorsStatusTable,
    MastGimbalControls,
    DriveMoteusStateTable,
    PDBFuse,
    //   SAArmControls,
    LimitSwitch,
    CalibrationCheckbox,
    //   CommReadout,
    //   MCUReset,
    MotorAdjust,
    OdometryReading
  },
  data() {
    return {
      // Default coordinates at MDRS
      odom: {
        latitude_deg: 38.406025,
        longitude_deg: -110.7923723,
        bearing_deg: 0,
        altitude: 0
      },

      jointState: {},

      moteusState: {
        name: [] as string[],
        error: [] as string[],
        state: [] as string[],
        limit_hit:
          [] as boolean[] /* Each motor stores an array of 4 indicating which limit switches are hit */
      },

      motorData: {
        name: [] as string[],
        position: [] as number[],
        velocity: [] as number[],
        effort: [] as number[]
      }
    }
  },

  created: function () {
    //   disableAutonLED(this.$ros);
    //   this.odom_sub = new ROSLIB.Topic({
    //     ros: this.$ros,
    //     name: "/gps/fix",
    //     messageType: "sensor_msgs/NavSatFix"
    //   });
    //   this.odom_sub.subscribe((msg) => {
    //     // Callback for latLng to be set
    //     this.odom.latitude_deg = msg.latitude;
    //     this.odom.longitude_deg = msg.longitude;
    //   });
    //   this.tfClient = new ROSLIB.TFClient({
    //     ros: this.$ros,
    //     fixedFrame: "odom",
    //     // Thresholds to trigger subscription callback
    //     angularThres: 0.0001,
    //     transThres: 0.01
    //   });
    //   // Subscriber for odom to base_link transform
    //   this.tfClient.subscribe("base_link", (tf) => {
    //     // Callback for IMU quaternion that describes bearing
    //     this.odom.bearing_deg = quaternionToMapAngle(tf.rotation);
    //   });
    //   this.brushless_motors_sub = new ROSLIB.Topic({
    //     ros: this.$ros,
    //     name: "drive_status",
    //     messageType: "mrover/MotorsStatus"
    //   });
    //   this.brushless_motors_sub.subscribe((msg) => {
    //     this.jointState = msg.joint_states;
    //     this.moteusState = msg.moteus_states;
    //   });
  }
}
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: repeat(3, auto);
  grid-template-rows: auto 50vh repeat(3, auto);
  grid-template-areas:
    'header header header'
    'map map waypoints'
    'odom cameras cameras'
    'arm limit calibration'
    'pdb moteus jointState';
  font-family: sans-serif;
  height: auto;
}

.header {
  grid-area: header;
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px;
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

.calibration-checkboxes {
  margin: -4% 0 1% 0;
}

ul#vitals li {
  display: inline;
  float: left;
  padding: 0px 10px 0px 0px;
}
</style>
