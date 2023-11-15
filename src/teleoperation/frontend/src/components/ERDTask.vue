<template>
    <PDBFuse/>
    <DriveMoteusStateTable :moteus-state-data="moteusState" />
    <ArmMoteusStateTable/>
    <JoystickValues/>
    <DriveControls></DriveControls>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import PDBFuse from "./PDBFuse.vue";
import DriveMoteusStateTable from "./DriveMoteusStateTable.vue";
import JoystickValues from './JoystickValues.vue';
import ArmMoteusStateTable from "./ArmMoteusStateTable.vue";
import DriveControls from './DriveControls.vue';

export default defineComponent({
  components : {
    PDBFuse,
    DriveMoteusStateTable,
    ArmMoteusStateTable,
    JoystickValues,
    DriveControls
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