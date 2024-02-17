<template>
  <div class="wrap box">
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
export default {
  props: {
    name: {
      type: String,
      required: true
    },
    id: {
      type: Number,
      required: true
    },
    stream: {
      type: Number,
      required: true
    }
  },
  data() {
    return {
      selectedQuality: '2',
      selectedStream: this.stream,
      prevStream: this.stream,
      numQuality: 5
    }
  },

  watch: {
    stream: function () {
      this.prevStream = this.stream
      this.selectedStream = this.stream
    }
  },

  created: function () {
    //   var arg = new ROSLIB.Param({
    //     ros: this.$ros,
    //     name: "cameras/max_num_resolutions"
    //   });
    //   arg.get((value) => {
    //     this.numQuality = value;
    //   });
  },

  methods: {
    changeQuality: function () {
      this.$emit('newQuality', {
        index: this.id,
        value: parseInt(this.selectedQuality)
      })
    },

    swapStream() {
      this.$emit('swapStream', {
        prev: this.prevStream,
        newest: this.selectedStream
      })
      this.prevStream = this.selectedStream
    }
  }
}
</script>

<style scoped>
/* .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
  }
  .wrap {
    margin: 10px;
    padding: 10px;
  }
  
  .wrap > * {
    margin: 5px 0 5px 0;
  } */
</style>
