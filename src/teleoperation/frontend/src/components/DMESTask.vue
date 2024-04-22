<template>
  <div :class="type === 'ES' ? 'wrapper-es' : 'wrapper-dm'">
    <div class="shadow p-3 mb-5 header">
      <h1 v-if="type === 'ES'">ES GUI Dashboard</h1>
      <h1 v-else>DM GUI Dashboard</h1>
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

    <div class="shadow p-3 rounded cameras">
      <Cameras :isSA="false" :mission="'ik'" />
    </div>
    <div v-if="type === 'DM'" class="shadow p-3 rounded odom">
      <OdometryReading :odom="odom" />
    </div>
    <div v-if="type === 'DM'" class="shadow p-3 rounded map">
      <BasicMap :odom="odom" />
    </div>
    <div class="shadow p-3 rounded pdb">
      <PDBFuse />
    </div>
    <div class="shadow p-3 rounded drive-vel-data">
      <MotorsStatusTable :motorData="motorData" :vertical="true" />
    </div>
    <div v-if="type === 'DM'" class="shadow p-3 rounded waypoint-editor">
      <BasicWaypointEditor :odom="odom" />
    </div>
    <div>
      <DriveControls></DriveControls>
    </div>
    <div class="shadow p-3 rounded arm-controls">
      <ArmControls />
    </div>
    <div class="shadow p-3 rounded moteus">
      <DriveMoteusStateTable :moteus-state-data="moteusDrive" />
      <ArmMoteusStateTable />
    </div>
    <div v-show="false">
      <MastGimbalControls></MastGimbalControls>
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import { mapState, mapActions } from 'vuex'
import PDBFuse from './PDBFuse.vue'
import DriveMoteusStateTable from './DriveMoteusStateTable.vue'
import ArmMoteusStateTable from './ArmMoteusStateTable.vue'
import ArmControls from './ArmControls.vue'
import BasicMap from './BasicRoverMap.vue'
import BasicWaypointEditor from './BasicWaypointEditor.vue'
import Cameras from './Cameras.vue'
import MotorsStatusTable from './MotorsStatusTable.vue'
import OdometryReading from './OdometryReading.vue'
import DriveControls from './DriveControls.vue'
import MastGimbalControls from './MastGimbalControls.vue'
import type ArmMoteusStateTableVue from './ArmMoteusStateTable.vue'

export default defineComponent({
  components: {
    PDBFuse,
    DriveMoteusStateTable,
    ArmMoteusStateTable,
    ArmControls,
    BasicMap,
    BasicWaypointEditor,
    Cameras,
    MotorsStatusTable,
    OdometryReading,
    DriveControls,
    MastGimbalControls
  },

  props: {
    type: {
      type: String,
      required: true
    }
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

      moteusDrive: {
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
        effort: [] as number[],
        state: [] as string[],
        error: [] as string[]
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
        this.moteusDrive.name = msg.name
        this.moteusDrive.state = msg.state
        this.moteusDrive.error = msg.error
        this.moteusDrive.limit_hit = msg.limit_hit
      } else if (msg.type == 'center_map') {
        this.odom.latitude_deg = msg.latitude
        this.odom.longitude_deg = msg.longitude
      }
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
    cancelIK: function () {
      this.sendMessage({ type: 'cancel_click_ik' })
    }
  },

  created: function () {
    window.setTimeout(() => {
      this.sendMessage({ type: 'center_map' })
    }, 250)
    window.addEventListener('keydown', (event: KeyboardEvent) => {
      if (event.key === ' ') {
        this.cancelIK(event)
      }
    })
  }
})
</script>

<style>
.wrapper-dm {
  display: grid;
  gap: 10px;
  grid-template-columns: 50% 50%;
  grid-template-rows: repeat(6, auto);
  grid-template-areas:
    'header header'
    'map waypoint-editor'
    'map odom'
    'arm-controls drive-vel-data'
    'moteus pdb'
    'cameras cameras';
  font-family: sans-serif;
  height: auto;
}

.wrapper-es {
  display: grid;
  gap: 10px;
  grid-template-columns: repeat(2, auto);
  grid-template-rows: repeat(4, auto);
  grid-template-areas:
    'header header'
    'drive-vel-data arm-controls'
    'pdb moteus'
    'cameras cameras';
  font-family: sans-serif;
  height: auto;
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
}
</style>
