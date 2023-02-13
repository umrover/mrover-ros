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
        :header="'Drive Motors'"
      ></JointStateTable>
    </div>
    <div class="box arm-jointstate light-bg">
      <JointStateTable
        :joint-state-data="armjointState"
        :vertical="true"
        :header="'Arm Motors'"
      ></JointStateTable>
    </div>
    <div class="box drive-moteus light-bg">
      <MoteusStateTable
        :moteus-state-data="drivemoteusState"
        :motor="true"
        :header="'Drive Moteus'"
      ></MoteusStateTable>
    </div>
    <div class="box arm-moteus light-bg">
      <MoteusStateTable
        :moteus-state-data="armmoteusState"
        :header="'Arm Moteus'"
      ></MoteusStateTable>
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
    <div class="box velocity light-bg">
      <Velocity></Velocity>
    </div>
    <div class="box odometry light-bg">
      <Odom :odom="odom"></Odom>
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
import Odom from "./OdometryReading.vue";
import Velocity from "./VelocityCommand.vue";

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
    Odom,
    Velocity
  },
  data() {
    return {
      primary: false,

      // Default object isn't empty, so has to be initialized to ""
      drivemoteusState: {
        name: ["", "", "", "", "", ""],
        error: ["", "", "", "", "", ""],
        state: ["", "", "", "", "", ""]
      },

      armmoteusState: {
        name: ["", "", "", "", "", ""],
        error: ["", "", "", "", "", ""],
        state: ["", "", "", "", "", ""]
      },

      motorjointState: {},

      armjointState: {},

      odom: {
        latitude_deg: 42.294864932393835,
        longitude_deg: -83.70781314674628,
        bearing_deg: 0,
        speed: 0
      },

      odom_sub: null
    };
  },

  created: function () {
    this.arm_JointState = new ROSLIB.Topic({
      ros: this.$ros,
      name: "ra_status",
      messageType: "mrover/MotorsStatus"
    });

    this.brushless_motors = new ROSLIB.Topic({
      ros: this.$ros,
      name: "drive_status",
      messageType: "mrover/MotorsStatus"
    });

    this.odom_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/gps/fix",
      messageType: "sensor_msgs/NavSatFix"
    });

    this.brushless_motors.subscribe((msg) => {
      this.motorjointState = msg.joint_states;
      this.drivemoteusState = msg.moteus_states;
    });

    this.arm_JointState.subscribe((msg) => {
      this.armjointState = msg.joint_states;
      this.armmoteusState = msg.moteus_states;
    });

    this.odom_sub.subscribe((msg) => {
      // Callback for latLng to be set
      this.odom.latitude_deg = msg.latitude;
      this.odom.longitude_deg = msg.longitude;
    });
  },

  methods: {
    primaryEnabled: function (enabled) {
      this.primary = enabled;
    }
  }
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
    "drive-moteus cameras"
    "arm-jointstate drive-vel-data"
    "velocity odometry"
    "arm-moteus arm-moteus";

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

.drive-moteus {
  grid-area: drive-moteus;
}

.arm-jointstate {
  grid-area: arm-jointstate;
}

.velocity {
  grid-area: velocity;
}

.odometry {
  grid-area: odometry;
}

.arm-moteus {
  grid-area: arm-moteus;
}
</style>
