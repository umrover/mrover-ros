<template>
  <div class='wrapper'>
    <div class='shadow p-3 mb-5 header'>
      <h1>SA Dashboard</h1>
    </div>
    <div class='shadow p-3 rounded map'>
      <BasicMap :odom='odom' />
    </div>
    <div class='shadow p-3 rounded waypoints'>
      <BasicWaypointEditor :odom='odom' />
    </div>
    <div class='shadow p-3 rounded cameras'>
      <Cameras :isSA='true' :mission="'sa'" />
    </div>
    <div class='shadow p-3 rounded soildata'>
      <SoilData />
    </div>
    <div>
      <DriveControls />
    </div>
    <div class='shadow p-3 rounded arm'>
      <SAArmControls />
    </div>
    <div class='shadow p-3 rounded pdb'>
      <PDBFuse />
    </div>
    <div class='shadow p-3 rounded moteus'>
      <ControllerDataTable msg-type='drive_state' header='Drive States' />
    </div>
    <div class='shadow p-3 rounded limit'>
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
    <div class='shadow p-3 rounded calibration'>
      <h3>Calibrations</h3>
      <br />
      <div class='calibration-checkboxes'>
        <button class='btn btn-primary my-5' @click='resetGimbal()'>Reset Gimbal</button>
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
    </div>
    <div v-show='false'>
      <MastGimbalControls />
    </div>
    <div class='shadow p-3 rounded odom'>
      <OdometryReading :odom='odom'></OdometryReading>
    </div>
  </div>
</template>

<script lang='ts'>
import BasicMap from './BasicRoverMap.vue'
import SoilData from './SoilData.vue'
import BasicWaypointEditor from './BasicWaypointEditor.vue'
import DriveControls from './DriveControls.vue'
import MastGimbalControls from './MastGimbalControls.vue'
import PDBFuse from './PDBFuse.vue'
import Cameras from './Cameras.vue'
import LimitSwitch from './LimitSwitch.vue'
import CalibrationCheckbox from './CalibrationCheckbox.vue'
import MotorAdjust from './MotorAdjust.vue'
import OdometryReading from './OdometryReading.vue'
import ControllerDataTable from './ControllerDataTable.vue'
import SAArmControls from './SAArmControls.vue'
import { quaternionToMapAngle } from '../utils.js'
import { mapActions, mapState } from 'vuex'

let interval: number

export default {
  components: {
    ControllerDataTable,
    BasicMap,
    SoilData,
    BasicWaypointEditor,
    Cameras,
    DriveControls,
    MastGimbalControls,
    DriveMoteusStateTable: ControllerDataTable,
    PDBFuse,
    SAArmControls,
    LimitSwitch,
    CalibrationCheckbox,
    //   CommReadout,
    //   MCUReset,
    MotorAdjust,
    OdometryReading,
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
    ...mapState('websocket', ['message'])
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
      } else if (msg.type == 'nav_sat_fix') {
        this.odom.latitude_deg = msg.latitude
        this.odom.longitude_deg = msg.longitude
        this.odom.altitude = msg.altitude
      } else if (msg.type == 'orientation') {
        this.odom.bearing_deg = quaternionToMapAngle(msg.rotation)
      }
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    resetGimbal: function() {
      this.sendMessage({ type: 'reset_gimbal' })
    }
  },

  created: function() {
    interval = setInterval(() => {
      this.sendMessage({ type: 'orientation' })
    }, 1000)
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
