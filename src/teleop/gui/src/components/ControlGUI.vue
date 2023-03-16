<template>
  <div>
    <CommReadout class="comm"></CommReadout>
    <DriveControls></DriveControls>
    <ArmControls></ArmControls>
    <MastGimbalControls></MastGimbalControls>
    <JointStateTable
      :joint-state-data="jointState"
      :vertical="true"
    ></JointStateTable>
    <MoteusStateTable :moteus-state-data="moteusState"></MoteusStateTable>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import DriveControls from "./DriveControls.vue";
import ArmControls from "./ArmControls.vue";
import MastGimbalControls from "./MastGimbalControls.vue";
import JointStateTable from "./JointStateTable.vue";
import MoteusStateTable from "./MoteusStateTable.vue";
import CommReadout from "./CommReadout.vue";

export default {
  components: {
    DriveControls,
    ArmControls,
    JointStateTable,
    MastGimbalControls,
    MoteusStateTable,
    CommReadout
  },
  data() {
    return {
      jointState: {},
      moteusState: {},
    };
  },

  created: function () {
    this.brushless_motors = new ROSLIB.Topic({
      ros: this.$ros,
      name: "drive_status",
      messageType: "mrover/MotorsStatus",
    });

    this.brushless_motors.subscribe((msg) => {
      this.jointState = msg.joint_states;
      this.moteusState = msg.moteus_states;
    });
  },
};
</script>
