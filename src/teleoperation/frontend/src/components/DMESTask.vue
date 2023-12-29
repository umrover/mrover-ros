<template>
  <div :class="type === 'ES' ? 'wrapper-es' : 'wrapper-dm'">
    <div class="shadow p-3 mb-5 header">
      <h1 v-if="type === 'ES'">ES GUI Dashboard</h1>
      <h1 v-else>DM GUI Dashboard</h1>
      <img class="logo" src="mrover.png" alt="MRover" title="MRover" width="200" />
      <div class="help">
        <img src="help.png" alt="Help" title="Help" width="48" height="48" />
      </div>
      <div class="helpscreen"></div>
      <div class="helpimages" style="
          display: flex;
          align-items: center;
          justify-content: space-evenly;
        ">
        <img src="joystick.png" alt="Joystick" title="Joystick Controls"
          style="width: auto; height: 70%; display: inline-block" />
      </div>
    </div>

    <div class="shadow p-3 rounded cameras">
      <Cameras :primary="true" />
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
      <JointStateTable :joint-state-data="jointState" :vertical="true" />
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
      <DriveMoteusStateTable :moteus-state-data="moteusState" />
      <ArmMoteusStateTable />
    </div>
    <!-- <div v-show="false">
      <MastGimbalControls></MastGimbalControls>
    </div> -->
  </div>
</template>

<script lang="ts">
import { defineComponent, inject } from 'vue'
import { mapState } from 'vuex';
import PDBFuse from "./PDBFuse.vue";
import DriveMoteusStateTable from "./DriveMoteusStateTable.vue";
import ArmMoteusStateTable from "./ArmMoteusStateTable.vue";
import ArmControls from "./ArmControls.vue";
import BasicMap from "./BasicRoverMap.vue";
import BasicWaypointEditor from './BasicWaypointEditor.vue';
import Cameras from './Cameras.vue';
import JointStateTable from "./JointStateTable.vue";
import OdometryReading from "./OdometryReading.vue";
import DriveControls from './DriveControls.vue';

export default defineComponent({
  components: {
    PDBFuse,
    DriveMoteusStateTable,
    ArmMoteusStateTable,
    ArmControls,
    BasicMap,
    BasicWaypointEditor,
    Cameras,
    JointStateTable,
    OdometryReading,
    DriveControls
  },

  props: {
    type: {
      type: String,
      required: true
    }
    
  },

  data() {
    return {
      // websocket: inject("webSocketService") as WebSocket,
      // websocket: new WebSocket('ws://localhost:8000/ws/gui'),
      // Default coordinates at MDRS
      odom: {
        latitude_deg: 38.4060250,
        longitude_deg: -110.7923723,
        bearing_deg: 0,
        altitude: 0,
        speed: 0
      },
      
      // Default object isn't empty, so has to be initialized to ""
      moteusState: {
        name: ["", "", "", "", "", ""],
        error: ["", "", "", "", "", ""],
        state: ["", "", "", "", "", ""]
      },

      jointState: {
        name: [],
        position: [],
        velocity: [],
        effort: []
      }
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == "joint_state") {
        this.jointState.name = msg.name;
        this.jointState.position = msg.position;
        this.jointState.velocity = msg.velocity;
        this.jointState.effort = msg.effort;
      }
    }
  }

  // created() {
  //   this.websocket.onmessage = (event) => {
  //     const msg = JSON.parse(event.data)
  //     if (msg.type == "joint_state") {
  //       this.jointState.name = msg.name;
  //       this.jointState.position = msg.position;
  //       this.jointState.velocity = msg.velocity;
  //       this.jointState.effort = msg.effort;
  //     }
  //   }
  // }
})
</script>

<style>
.wrapper-dm {
  display: grid;
  gap: 10px;
  grid-template-columns: 50% 50%;
  grid-template-rows: repeat(6, auto);
  grid-template-areas:
    "header header"
    "map waypoint-editor"
    "map odom"
    "map cameras"
    "arm-controls drive-vel-data"
    "moteus pdb";
  font-family: sans-serif;
  height: auto;
}

.wrapper-es {
  display: grid;
  gap: 10px;
  grid-template-columns: repeat(2, auto);
  ;
  grid-template-rows: repeat(4, auto);
  ;
  grid-template-areas:
    "header header"
    "cameras arm-controls"
    "drive-vel-data moteus"
    "pdb pdb";
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

.help:hover~.helpscreen,
.help:hover~.helpimages {
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