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
      <h1>ESW Debug</h1>
      <div class="spacer"></div>
      <div class="spacer"></div>
    </div>
    <DriveControls></DriveControls>
    <div class="box arm-controls light-bg">
      <ArmControls />
    </div>
    <GimbalControls></GimbalControls>
    <div class="box drive-vel-data light-bg">
      <JointStateTable
        :joint-state-data="motorjointState"
        :vertical="true"
        :header = "'Drive Motors'"
      ></JointStateTable>
    </div>
    <div class="box arm-jointstate light-bg">
      <JointStateTable
        :joint-state-data="armjointState"
        :vertical = "true"
        :header = "'Arm Motors'"
      ></JointStateTable>
    </div>
    <div class="box moteus light-bg">
      <MoteusStateTable :moteus-state-data="moteusState"></MoteusStateTable>
    </div>
    <div class="box pdb light-bg">
      <PDBFuse></PDBFuse>
    </div>
    <div class="box cameras light-bg">
      <Cameras :primary="primary"></Cameras>
      <Checkbox
        ref="Primary"
        :name="'Primary Computer?'"
        @toggle="primaryEnabled($event)"
      />
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import DriveControls from "./DriveControls.vue";
import ArmControls from "./ArmControls.vue";
import GimbalControls from "./GimbalControls.vue";
import JointStateTable from "./JointStateTable.vue";
import MoteusStateTable from "./MoteusStateTable.vue";
import PDBFuse from "./PDBFuse.vue";
import Cameras from "./Cameras.vue";
import Checkbox from "./Checkbox.vue";

export default {
  components: {
    DriveControls,
    ArmControls,
    GimbalControls,
    JointStateTable,
    MoteusStateTable,
    PDBFuse,
    Cameras,
    Checkbox,
  },
  data() {
    return {
      primary: false,

       // Default object isn't empty, so has to be initialized to ""
      moteusState: {
        name: ["", "", "", "", "", ""],
        error: ["", "", "", "", "", ""],
        state: ["", "", "", "", "", ""],
      },

     motorjointState: {},

     armjointState:{}
    };
  },

  created: function () {
    this.arm_JointState = new ROSLIB.Topic({
      ros: this.$ros,
      name: "ra_status",
      messageType: "mrover/MotorsStatus",
    });

    this.brushless_motors = new ROSLIB.Topic({
      ros: this.$ros,
      name: "drive_status",
      messageType: "mrover/MotorsStatus",
    });


    this.brushless_motors.subscribe((msg) => {
      this.motorjointState = msg.joint_states;
      this.moteusState = msg.moteus_states;
    });

    this.arm_JointState.subscribe((msg) => {
      this.armjointState = msg.joint_states;
    });
  },

  methods: {
    primaryEnabled: function (enabled) {
      this.primary = enabled;
    },
  },
};
</script>
<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: auto auto;
  grid-template-rows: 60px 250px auto auto auto auto;
  grid-template-areas:
    "header header"
    "arm-controls pdb"
    "moteus cameras"
    "arm-jointstate drive-vel-data";
    

  font-family: sans-serif;
  height: auto;
}

.header {
  grid-area: header;
  display: flex;
  align-items: center;
}
.cameras {
  grid-area: cameras;
}

.pdb {
  grid-area: pdb;
}

.drive-vel-data {
  grid-area: drive-vel-data;
}

.arm-controls {
  grid-area: arm-controls;
}

.moteus {
  grid-area: moteus;
}

.arm-jointstate {
  grid-area: arm-jointstate;
}
</style>
