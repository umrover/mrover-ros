<template>
  <div>
    <p>{{ name }} ID: {{ id }}</p>
    Stream:
    <input v-model="selectedStream" class="box" type="Number" min="0" max="3" />
    <button class="box" @click="swapStream()">Change stream</button>
    <label for="quality">Quality:</label>
    <select id="quality" v-model="selectedQuality" class="box" @change="changeQuality()">
      <option v-for="i in numQuality" :key="i">{{ i - 1 }}</option>
    </select>
  </div>
</template>
  
<script lang="ts">

import { mapActions, mapState } from "vuex";

export default {
  props: {
    name: {
      type: String,
      required: true,
    },
    id: {
      type: Number,
      required: true,
    },
    stream: {
      type: Number,
      required: true,
    },
  },
  data() {
    return {
      selectedQuality: "2",
      selectedStream: this.stream,
      prevStream: this.stream,
      numQuality: 5
    };
  },

  watch: {
    message(msg) {
      if (msg.type == "max_resolution") {
        this.numQuality = msg.res;
      }
    },
    stream: function () {
      console.log("changed");
      this.prevStream = this.stream;
      this.selectedStream = this.stream;
    },
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  created: function () {
    window.setTimeout(() => {
      this.sendMessage({ "type": "num_resolutions" });
    }, 250)
  },

  methods: {

    ...mapActions('websocket', ['sendMessage']),

    changeQuality: function () {
      this.$emit("newQuality", {
        index: this.id,
        value: parseInt(this.selectedQuality),
      });
    },

    swapStream() {
      this.$emit("swapStream", {
        prev: this.prevStream,
        newest: this.selectedStream,
      });
      this.prevStream = this.selectedStream;
    },
  },
};
</script>
  
<style scoped></style>