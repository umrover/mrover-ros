<template>
  <div class="wrapper">
    <div class="shadow p-3 mb-5 header">
      <img class="logo" src="/mrover.png" alt="MRover" title="MRover" width="200" />
      <h1>Auton Dashboard</h1>
      <!-- <MCUReset class="mcu_reset"></MCUReset>
        <CommReadout class="comms"></CommReadout> -->
      <div class="help">
        <img src="/help.png" alt="Help" title="Help" width="48" height="48" />
      </div>
      <div class="helpscreen"></div>
      <div class="helpimages" style="display: flex; align-items: center; justify-content: space-evenly">
        <img src="/joystick.png" alt="Joystick" title="Joystick Controls"
          style="width: auto; height: 70%; display: inline-block" />
      </div>
    </div>
    <div :class="['shadow p-3 rounded data', ledColor]">
      <h2>Nav State: {{ navState }}</h2>
      <!-- <div style="display: inline-block">
        <CameraFeed></CameraFeed>
      </div> -->
      <div style="display: inline-block; vertical-align: top">
        <p style="margin-top: 6px">Joystick Values</p>
        <JoystickValues />
      </div>
      <OdometryReading :odom="odom" />
    </div>
    <div class="shadow p-3 rounded map">
      <AutonRoverMap :odom="odom" />
    </div>
    <div class="shadow p-3 rounded waypoints">
      <AutonWaypointEditor :odom="odom" @toggleTeleop="teleopEnabledCheck = $event" />
    </div>
    <!--Enable the drive controls if auton is off-->
    <div v-if="!autonEnabled && teleopEnabledCheck" v-show="false" class="driveControls">
      <DriveControls />
    </div>
    <div v-show="false">
      <MastGimbalControls></MastGimbalControls>
    </div>
    <div class="conditions">
      <div v-if="!stuck_status" class="shadow p-3 rounded bg-success text-center">
        <h4>Nominal Conditions</h4>
      </div>
      <div v-else class="shadow p-3 rounded bg-danger text-center">
        <h4>Obstruction Detected</h4>
      </div>
    </div>
    <div class="shadow p-3 rounded cameras">
      <Cameras :primary="true" :isSA="false"/>
    </div>
    <div class="shadow p-3 rounded moteus">
      <DriveMoteusStateTable :moteus-state-data="moteusState" />
      <MotorsStatusTable :motor-data="motorData" :vertical="true" />
    </div>
  </div>
</template>

<script lang="ts">
import { mapActions, mapState, mapGetters } from 'vuex'
import DriveMoteusStateTable from './DriveMoteusStateTable.vue'
import AutonRoverMap from './AutonRoverMap.vue'
import AutonWaypointEditor from './AutonWaypointEditor.vue'
import CameraFeed from './CameraFeed.vue'
import Cameras from './Cameras.vue'
import MotorsStatusTable from './MotorsStatusTable.vue'
import OdometryReading from './OdometryReading.vue'
import JoystickValues from './JoystickValues.vue'
import DriveControls from './DriveControls.vue'
import MastGimbalControls from './MastGimbalControls.vue'
import { quaternionToMapAngle } from '../utils.js'
import { defineComponent } from 'vue'

let interval: number

export default defineComponent({
  components: {
    DriveMoteusStateTable,
    AutonRoverMap,
    AutonWaypointEditor,
    CameraFeed,
    Cameras,
    MotorsStatusTable,
    OdometryReading,
    JoystickValues,
    DriveControls,
    MastGimbalControls
  },

  // add prop where map has the center property and autontask sends it once it gets it

  data() {
    return {
      // Default coordinates are at MDRS
      odom: {
        latitude_deg: 42.293195,
        longitude_deg: -83.7096706,
        bearing_deg: 0,
        altitude: 0
      },

      teleopEnabledCheck: false,

      ledColor: 'bg-danger', //red

      stuck_status: false,

      navState: 'OffState',

      moteusState: {
        name: [] as string[],
        error: [] as string[],
        state: [] as string[],
        limit_hit: [] as boolean[] /* Each motor stores an array of 4 indicating which limit switches are hit */
      },

      motorData: {
        name: [] as string[],
        position: [] as number[],
        velocity: [] as number[],
        effort: [] as number[],
        state: [] as string[],
        error: [] as string[]
      }
    }
  },

  computed: {
    ...mapState('websocket', ['message']),

    ...mapGetters('autonomy', {
      autonEnabled: 'autonEnabled',
      teleopEnabled: 'teleopEnabled'
    })
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
      } else if (msg.type == 'led') {
        if (msg.red) this.ledColor = 'bg-danger' //red
        else if (msg.green) this.ledColor = 'blink' //blinking green
        else if (msg.blue) this.ledColor = 'bg-primary' //blue
      } else if (msg.type == 'nav_state') {
        this.navState = msg.state
      } else if (msg.type == 'nav_sat_fix') {
        this.odom.latitude_deg = msg.latitude
        this.odom.longitude_deg = msg.longitude
        this.odom.altitude = msg.altitude
      } else if (msg.type == 'auton_tfclient') {
        this.odom.bearing_deg = quaternionToMapAngle(msg.rotation)
      } else if (msg.type == "center_map") {
        this.odom.latitude_deg = msg.latitude
        this.odom.longitude_deg = msg.longitude
      }
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage'])
  },

  beforeUnmount: function () {
    this.ledColor = 'bg-white'
    window.clearInterval(interval)
  },

  created: function () {
    window.setTimeout(() => {
      this.sendMessage({ "type": "center_map" });
    }, 250)
      interval = setInterval(() => {
      this.sendMessage({ type: 'auton_tfclient' })
    }, 1000)
  },

})
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: 40% 20% auto;
  grid-template-rows: repeat(6, auto);
  grid-template-areas:
    'header header header'
    'map map waypoints'
    'data data waypoints'
    'data data conditions'
    'moteus moteus moteus'
    'cameras cameras cameras';

  font-family: sans-serif;
  height: auto;
  width: auto;
}

.blink {
  animation: blinkAnimation 1s infinite;
  /* Blinks green every second */
}

@keyframes blinkAnimation {
  0%,
  100% {
    background-color: var(--bs-success);
  }

  50% {
    background-color: var(--bs-white);
  }
}

.header {
  grid-area: header;
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px;
}

.logo {
  position: absolute;
  left: 50%;
  transform: translateX(-50%);
}

h2 {
  padding: 2px;
  margin: 0px;
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

/* Grid area declarations */
.map {
  grid-area: map;
}
.waypoints {
  grid-area: waypoints;
}

.conditions {
  grid-area: conditions;
}

.cameras {
  grid-area: cameras;
}

.moteus {
  grid-area: moteus;
}

.data {
  grid-area: data;
}
</style>
