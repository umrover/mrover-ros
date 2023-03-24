<template>
  <div>
    <div class="page_header">
      <img
        src="/static/new_mrover.png"
        alt="MRover"
        title="MRover"
        width="185"
        height="53"
      />
      <h1>Temp Controls</h1>
    </div>
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
import "../assets/style.css";

export default {
  components: {
    DriveControls,
    ArmControls,
    JointStateTable,
    MastGimbalControls,
    MoteusStateTable,
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
