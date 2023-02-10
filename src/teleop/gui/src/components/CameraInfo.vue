<template>
    <div class="box">
        <p>{{name}} ID: {{id}}</p>
        <div class="section">
        Stream: <input class="box" type='Number' min="0" max="3" v-model ='selectedStream'>
        <button class="button" v-on:click="swapStream()">Change stream</button>
        </div>
        <div class="section">
        <label for="quality">Quality:</label>
        <select class="box" id="quality" v-model="selectedQuality" @change="changeQuality()">
          <option value="0">Low</option>
          <option value="1">Medium</option>
          <option value="2">High</option>
        </select>
        </div>
    </div>
  </template>
  
  <script>
  import '../assets/style.css';
  
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
      selectedQuality: "0",
      selectedStream: this.stream,
      prevStream: this.stream,
    };
  },

  watch: {
    stream: function () {
      this.prevStream = this.stream;
      this.selectedStream = this.stream;
    },
  },

  methods: {
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

<style scoped>
.wrap {
  margin: 10px;
  padding: 10px;
}

.section {
  display: inline-block;
}

.wrap > * {
  margin: 5px 0 5px 0;
}

.button {
  margin: 2px;
}
</style>
