<template>
  <PDBFuse />
  <JointStateTable :jointStateData="jointState" :vertical="true" />
  <DriveMoteusStateTable :moteus-state-data="moteusState" />
  <ArmMoteusStateTable />
</template>

<script lang="ts">
import { defineComponent, inject } from 'vue'
import PDBFuse from "./PDBFuse.vue";
import DriveMoteusStateTable from "./DriveMoteusStateTable.vue";
import ArmMoteusStateTable from "./ArmMoteusStateTable.vue";
import JointStateTable from "./JointStateTable.vue";

export default defineComponent({
  components: {
    PDBFuse,
    DriveMoteusStateTable,
    ArmMoteusStateTable,
    JointStateTable
  },

  data() {
    return {
      websocket: inject("webSocketService") as WebSocket,
      //websocket: new WebSocket("ws://localhost:8080/ws/g"),
      // Default object isn't empty, so has to be initialized to ""
      moteusState: {
        name: ["", "", "", "", "", ""],
        error: ["", "", "", "", "", ""],
        state: ["", "", "", "", "", ""]
      },
      jointState: {}
    }
  },
  created() {
    this.websocket.onopen = () => {
      console.log("Hejej")
    }
    this.websocket.onmessage = (event) => {
      console.log("HI")
      const msg = JSON.parse(event.data)
      if (msg.type == "joint_state") {
        console.log(msg.type)
        // this.jointState.name = msg.name;
        // this.jointState.position = msg.position;
        // this.jointState.velocity = msg.velocity;
        // this.jointState.effort = msg.effort;
      }
    }
  }
})
</script>

<style></style>