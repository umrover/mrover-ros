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
    <DriveMoteusStateTable :moteus-state-data="moteusState"></DriveMoteusStateTable>
    <FlightAttitudeIndicator></FlightAttitudeIndicator>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import DriveControls from "./DriveControls.vue";
import ArmControls from "./ArmControls.vue";
import MastGimbalControls from "./MastGimbalControls.vue";
import JointStateTable from "./JointStateTable.vue";
import DriveMoteusStateTable from "./DriveMoteusStateTable.vue";
import CommReadout from "./CommReadout.vue";
import FlightAttitudeIndicator from "./FlightAttitudeIndicator.vue";

export default {
  components: {
    DriveControls,
    ArmControls,
    JointStateTable,
    MastGimbalControls,
    DriveMoteusStateTable,
    CommReadout,
    FlightAttitudeIndicator
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
