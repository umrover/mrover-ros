<template>
  <div>
    <p>pitch: {{pitch}}</p>
    <p>roll: {{roll}}</p>
    <Attitude :size="200" :pitch="pitch" :roll="roll"></Attitude>
  </div>
</template>

<script lang="ts">
import { Attitude } from "vue-flight-indicators";
import { mapState } from "vuex";

export default {
  components: {
    Attitude
  },
  data() {
    return {
      // Pitch and Roll in Degrees
      pitch: 0,
      roll: 0,
      // websocket: new WebSocket("ws://localhost:8000/ws/gui")
    };
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == "flight_attitude") {
        this.pitch = msg.pitch;
        this.roll = msg.roll;
      }
    }
  }
};
</script>

<style scoped></style>