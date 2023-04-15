<template>
  <div class="wrapper box">
    <p>{{ name }} ID: {{ id }}</p>
    <div>
      Stream:
      <input
        v-model="selectedStream"
        class="box"
        type="Number"
        min="0"
        max="3"
      />
      <button class="button" @click="swapStream()">Change stream</button>
    </div>
    <label for="quality">Quality:</label>
    <select
      id="quality"
      v-model="selectedQuality"
      class="box"
      @change="changeQuality()"
    >
    <option v-for="i in numQuality">{{i-1}}</option>
    </select>
  </div>
</template>

<script>
import "../assets/style.css";
import ROSLIB from 'roslib/src/RosLib';

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
      numQuality: 0
    };
  },

  watch: {
    stream: function () {
      this.prevStream = this.stream;
      this.selectedStream = this.stream;
    },
  },

  created: function() {
    var arg = new ROSLIB.Param({
      ros: this.$ros,
      name: "cameras/arguments"
    });
    arg.get((arr) => {
      this.numQuality = arr.length;
    });
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
.wrapper {
  /* wrap acts weird with flex from parent... */
  margin: 10px;
  padding: 10px;
}

.wrapper > * {
  margin: 5px 0 5px 0;
}

.button {
  margin: 2px;
}
</style>
