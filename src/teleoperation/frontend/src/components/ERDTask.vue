<template>
    <PDBFuse/>
    <DriveMoteusStateTable :moteus-state-data="moteusState" />
    <ArmMoteusStateTable/>
</template>

<script lang="ts">
import ROSLIB from "roslib";
import { defineComponent } from 'vue'
import PDBFuse from "./PDBFuse.vue";
import DriveMoteusStateTable from "./DriveMoteusStateTable.vue";
import ArmMoteusStateTable from "./ArmMoteusStateTable.vue";

export default defineComponent({
  components : {
    PDBFuse,
    DriveMoteusStateTable,
    ArmMoteusStateTable
  },

  data() {
    return {
      // Default object isn't empty, so has to be initialized to ""
      moteusState: {
        name: ["", "", "", "", "", ""],
        error: ["", "", "", "", "", ""],
        state: ["", "", "", "", "", ""]
      },
    }
  },

  created: function() {
    this.odom_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/gps/fix",
      messageType: "sensor_msgs/NavSatFix"
    });

    this.odom_sub.subscribe((msg) => {
      this.odom.altitude = msg.altitude;
    });
  }
})
</script>

<style>

</style>