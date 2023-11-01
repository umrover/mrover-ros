<template>
  <PDBFuse />
  <JointStateTable :jointStateData="JointState" :vertical="true" />
  <DriveMoteusStateTable :moteus-state-data="moteusState" />
  <ArmMoteusStateTable />
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import PDBFuse from "./PDBFuse.vue";
import DriveMoteusStateTable from "./DriveMoteusStateTable.vue";
import ArmMoteusStateTable from "./ArmMoteusStateTable.vue";

import JointStateTable from "./JointStateTable.vue"

export default defineComponent({
  components: {
    PDBFuse,
    DriveMoteusStateTable,
    ArmMoteusStateTable,
    JointStateTable
  },

  data() {
    return {
      // Default object isn't empty, so has to be initialized to ""
      moteusState: {
        name: ["", "", "", "", "", ""],
        error: ["", "", "", "", "", ""],
        state: ["", "", "", "", "", ""]
      },
      JointState: {
        name: [""],
        position: [""],
        velocity: [""],
        effort: [0]
      }
    }
  }
})
</script>

<style></style>