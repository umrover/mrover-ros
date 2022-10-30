<template>
  <div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>ERD Dashboard</h1>
      <div class="spacer"></div>
      <div class="help">
        <img src="/static/help.png" alt="Help" title="Help" width="48" height="48" />
      </div>
      <div class="helpscreen"></div>
      <div class="helpimages" style="display: flex; align-items: center; justify-content: space-evenly">
        <img v-if="controlMode === 'arm'" src="/static/arm.png" alt="Robot Arm" title="Robot Arm Controls" style="width: auto; height: 70%; display: inline-block" />
        <img v-else-if="controlMode === 'soil_ac'" src="/static/soil_ac.png" alt="Soil Acquisition" title="Soil Acquisition Controls" style="width: auto; height: 70%; display: inline-block" />
        <img src="/static/joystick.png" alt="Joystick" title="Joystick Controls" style="width: auto; height: 70%; display: inline-block" />
      </div>
    </div>
    <div class="box cameras light-bg">
      <Cameras/>
    </div>
    <div class="box arm-controls light-bg">
      <ArmControls/>
    </div>
    <div class="box odom light-bg">
      <OdometryReading v-bind:odom="odom"/>
    </div>
    <div class="box map light-bg">
      <ERDMap v-bind:odom="odom"/>
    </div>
    <div class="box drive light-bg">
      <DriveControls/>
    </div>
    <div class="box pdb light-bg">
      <PDBFuse/>
    </div>
    <div class="box drive-vel-data light-bg">
      <DriveVelData/>
    </div>
  </div>
</template>

<script>
import { mapGetters } from 'vuex'
import ArmControls from './ArmControls.vue'
import Cameras from './Cameras.vue'
import ERDMap from './ERDRoverMap.vue'
import OdometryReading from './OdometryReading.vue'
import DriveControls from './DriveControls.vue'
import PDBFuse from './PDBFuse.vue'
import DriveVelData from './DriveVelDataV.vue'

export default {
  data() {
    return {
      odom: {
        latitude_deg: 38,
        latitude_min: 24.38226,
        longitude_deg: -110,
        longitude_min: -47.51724,
        bearing_deg: 0,
        speed: 0
      }
    }
  },

  computed: {
    ...mapGetters('controls', {
      controlMode: 'controlMode'
    }),
  },

  components: {
    ERDMap,
    ArmControls,
    Cameras,
    OdometryReading,
    DriveControls,
    PDBFuse,
    DriveVelData
  }
}
</script>

<style scoped>
  .wrapper {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: auto auto;
    grid-template-rows: 60px 250px 150px auto;
    grid-template-areas: "header header"
                         "map cameras"
                         "map arm-controls"
                         "odom drive"
                         "pdb drive-vel-data";
    font-family: sans-serif;
    height: auto;
  }

  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
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

  .spacer {
    flex-grow: 0.8;
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
    opacity: 1.0;
    cursor: pointer;
  }

  .help:hover ~ .helpscreen, .help:hover ~ .helpimages {
    visibility: visible;
  }

  .map {
    grid-area: map;
  }

  .arm-controls {
    grid-area: arm-controls;
  }

  .cameras {
    grid-area: cameras;
  }

  .odom {
    grid-area: odom;
  }

  .drive {
    grid-area: drive;
  }

  .pdb {
    grid-area: pdb;
  }

  .drive-vel-data {
    grid-area: drive-vel-data;
  }

</style>
