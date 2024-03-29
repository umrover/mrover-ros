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
      <Cameras :primary="true" :isSA="true" />
    </div>
    <div class="shadow p-3 rounded soildata">
      <SoilData />
    </div>
    <div>
      <DriveControls />
    </div>
    <div class="shadow p-3 rounded arm">
      <SAArmControls />
    </div>
    <div class="shadow p-3 rounded pdb">
      <PDBFuse />
    </div>
    <div class="shadow p-3 rounded motorData">
      <MotorsStatusTable :motor-data="motorData" :vertical="true" />
    </div>
    <div class="shadow p-3 rounded moteus">
      <DriveMoteusStateTable :moteus-state-data="moteusState" />
    </div>
    <div class="shadow p-3 rounded limit">
      <h3>Limit Switches</h3>
      <LimitSwitch :service_name="'sa_enable_limit_switch_sa_x'" :display_name="'SA X Switch'" />
      <LimitSwitch :service_name="'sa_enable_limit_switch_sa_y'" :display_name="'SA Y Switch'" />
      <LimitSwitch :service_name="'sa_enable_limit_switch_sa_z'" :display_name="'SA Z Switch'" />
      <LimitSwitch
        :service_name="'sa_enable_limit_switch_sampler'"
        :display_name="'Sampler Switch'"
      />
      <LimitSwitch
        :service_name="'sa_enable_limit_switch_sensor_actuator'"
        :display_name="'Sensor Actuator Switch'"
      />
    </div>
    <div class="shadow p-3 rounded calibration">
      <h3>Calibrations</h3>
      <br />
      <div class="calibration-checkboxes">
        <CalibrationCheckbox :name="'SA X Calibration'" :topic_name="'sa_calibrate_sa_x'" />
        <CalibrationCheckbox :name="'SA Y Calibration'" :topic_name="'sa_calibrate_sa_y'" />
        <CalibrationCheckbox :name="'SA Z Calibration'" :topic_name="'sa_calibrate_sa_z'" />
        <CalibrationCheckbox
          :name="'SA Sampler Calibration'"
          :topic_name="'sa_calibrate_sampler'"
        />
        <CalibrationCheckbox
          :name="'SA Sensor Actuator Calibration'"
          :topic_name="'sa_calibrate_sensor_actuator'"
        />
      </div>
      <!-- <MotorAdjust
        :options="[
          { name: 'sa_joint_1', option: 'Joint 1' },
          { name: 'sa_joint_2', option: 'Joint 2' },
          { name: 'sa_joint_3', option: 'Joint 3' }
        ]"
      /> -->
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
import SoilData from './SoilData.vue'
import BasicWaypointEditor from './BasicWaypointEditor.vue'
import DriveControls from './DriveControls.vue'
import MastGimbalControls from './MastGimbalControls.vue'
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
import SAArmControls from './SAArmControls.vue'
import { disableAutonLED, quaternionToMapAngle } from '../utils.js'
import { mapState } from 'vuex'

export default {
  components: {
    BasicMap,
    SoilData,
    BasicWaypointEditor,
    Cameras,
    DriveControls,
    MastGimbalControls,
    DriveMoteusStateTable,
    PDBFuse,
    SAArmControls,
    LimitSwitch,
    CalibrationCheckbox,
    //   CommReadout,
    //   MCUReset,
    MotorAdjust,
    OdometryReading,
    MotorsStatusTable
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

      motorData: {
        name: [] as string[],
        position: [] as number[],
        velocity: [] as number[],
        effort: [] as number[],
        state: [] as string[],
        error: [] as string[]
      },
      // Moteus state table is set up to look for specific keys in moteusState so it can't be empty
      moteusState: {
        name: [] as string[],
        error: [] as string[],
        state: [] as string[],
        limit_hit:
          [] as boolean[] /* Each motor stores an array of 4 indicating which limit switches are hit */
      }
    }
  },

  computed: {
    ...mapState('websocket', ['message']),
  },

  watch: {
    message(msg) {
      if (msg.type == 'drive_status') {
        this.motorData.name = msg.name
        this.motorData.position = msg.position
        this.motorData.velocity = msg.velocity
        this.motorData.effort = msg.effort
        this.motorData.state = msg.state
        this.motorData.error = msg.error
      } else if (msg.type == 'drive_moteus') {
        this.moteusState.name = msg.name
        this.moteusState.state = msg.state
        this.moteusState.error = msg.error
        this.moteusState.limit_hit = msg.limit_hit
      } 
    }
  },

  created: function () {

  }
}
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: repeat(3, auto);
  grid-template-rows: auto 50vh repeat(4, 1fr);
  grid-template-areas:
    'header header header'
    'map map waypoints'
    'odom limit calibration'
    'arm limit calibration'
    'pdb moteus motorData'
    'pdb moteus soilData'
    'cameras cameras cameras';
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

.motorData {
  grid-area: motorData;
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

.soilData {
  grid-area: soilData;
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
