<template>
    <PDBFuse/>
    <DriveMoteusStateTable :moteus-state-data="moteusState" />
    <ArmMoteusStateTable/>
    <CameraFeed/>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import PDBFuse from "./PDBFuse.vue";
import DriveMoteusStateTable from "./DriveMoteusStateTable.vue";
import ArmMoteusStateTable from "./ArmMoteusStateTable.vue";
import CameraFeed from "./CameraFeed.vue";

export default defineComponent({
  components : {
    PDBFuse,
    DriveMoteusStateTable,
    ArmMoteusStateTable,
    CameraFeed
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
  }
})
</script>

<style>

</style>