<template>
  <div class='wrapper'>
    <div class='shadow p-3 mb-5 header'>
      <h1>SA Dashboard</h1>
      <div class="network">
      <NetworkMonitor />
      </div>
    </div>
    <div class='shadow p-3 rounded map'>
      <BasicMap :odom='odom' />
    </div>
    <div class='shadow p-3 rounded waypoints'>
      <BasicWaypointEditor :odom='odom' />
    </div>
    <div class='shadow p-3 rounded soilData'>
      <SoilData />
    </div>
    <div>
      <DriveControls />
    </div>
    <div class='shadow p-3 rounded arm'>
      <SAArmControls />
    </div>
    <div class='shadow p-3 rounded moteus'>
      <ControllerDataTable msg-type='drive_state' header='Drive States' />
    </div>
    <div class='shadow p-3 rounded joints'>
      <JointStateDataTable msg-type='sa_joint' header='SA Joints' />
      <JointStateDataTable msg-type='plunger' header='Plunger (w/o offset)' />
    </div>
    <!--    <div class='shadow p-3 rounded limit'>-->
    <!--      <h3>Limit Switches</h3>-->
    <!--      <LimitSwitch :service_name="'sa_enable_limit_switch_sa_x'" :display_name="'SA X Switch'" />-->
    <!--      <LimitSwitch :service_name="'sa_enable_limit_switch_sa_y'" :display_name="'SA Y Switch'" />-->
    <!--      <LimitSwitch :service_name="'sa_enable_limit_switch_sa_z'" :display_name="'SA Z Switch'" />-->
    <!--      <LimitSwitch-->
    <!--        :service_name="'sa_enable_limit_switch_sampler'"-->
    <!--        :display_name="'Sampler Switch'"-->
    <!--      />-->
    <!--      <LimitSwitch-->
    <!--        :service_name="'sa_enable_limit_switch_sensor_actuator'"-->
    <!--        :display_name="'Sensor Actuator Switch'"-->
    <!--      />-->
    <!--    </div>-->
    <!--    <div class='shadow p-3 rounded calibration'>-->
    <!--      <h3>Calibrations</h3>-->
    <!--      <br />-->
    <!--      <div class='calibration-checkboxes'>-->
    <!--        <button class='btn btn-primary my-5' @click='resetGimbal()'>Reset Gimbal</button>-->
    <!--        <CalibrationCheckbox :name="'SA X Calibration'" :topic_name="'sa_calibrate_sa_x'" />-->
    <!--        <CalibrationCheckbox :name="'SA Y Calibration'" :topic_name="'sa_calibrate_sa_y'" />-->
    <!--        <CalibrationCheckbox :name="'SA Z Calibration'" :topic_name="'sa_calibrate_sa_z'" />-->
    <!--        <CalibrationCheckbox-->
    <!--          :name="'SA Sampler Calibration'"-->
    <!--          :topic_name="'sa_calibrate_sampler'"-->
    <!--        />-->
    <!--        <CalibrationCheckbox-->
    <!--          :name="'SA Sensor Actuator Calibration'"-->
    <!--          :topic_name="'sa_calibrate_sensor_actuator'"-->
    <!--        />-->
    <!--      </div>-->
    <!--    </div>-->
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
import LimitSwitch from './LimitSwitch.vue'
import CalibrationCheckbox from './CalibrationCheckbox.vue'
import OdometryReading from './OdometryReading.vue'
import ControllerDataTable from './ControllerDataTable.vue'
import SAArmControls from './SAArmControls.vue'
import JointStateDataTable from './JointStateDataTable.vue'
import NetworkMonitor from "./NetworkMonitor.vue";
import { quaternionToMapAngle } from '../utils'
import { mapActions, mapState } from 'vuex'

export default {
  components: {
    ControllerDataTable,
    BasicMap,
    SoilData,
    BasicWaypointEditor,
    DriveControls,
    MastGimbalControls,
    SAArmControls,
    LimitSwitch,
    CalibrationCheckbox,
    NetworkMonitor,
    //   MCUReset,
    OdometryReading,
    JointStateDataTable
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
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      switch (msg.type) {
        case 'gps_fix':
          this.odom.latitude_deg = msg.latitude
          this.odom.longitude_deg = msg.longitude
          this.odom.altitude = msg.altitude
          break
        case 'orientation':
          this.odom.bearing_deg = quaternionToMapAngle(msg.orientation)
          break
      }
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage'])

    // resetGimbal: function() {
    //   this.sendMessage({ type: 'reset_gimbal' })
    // }
  }
}
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: 50% 50%;
  grid-template-areas:
    'header header'
    'arm soilData'
    'map waypoints'
    'map odom'
    'moteus joints';
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

.network {
  float: right;
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

.waypoints {
  grid-area: waypoints;
}

.arm {
  grid-area: arm;
}

.motorData {
  grid-area: motorData;
}

.moteus {
  grid-area: moteus;
}

.joints {
  grid-area: joints;
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
  padding: 0 10px 0 0;
}
</style>
