<template>
  <div class="wrap box">
    <p>{{ name }} ID: {{ id }}</p>
    Stream:
    <input v-model="selectedStream" class="box" type="Number" min="0" max="3" />
    <button class="box" @click="swapStream()">Change stream</button>
    <label for="quality">Quality:</label>
    <select
      id="quality"
      v-model="selectedQuality"
      class="box"
      @change="changeQuality()"
    >
    <option v-for="i in numQuality">{{i-1}}</option>
    </select>
    <button class="rounded button" @click="screenshot(id)">
        Screenshot
      </button>
  </div>
</template>

<script>
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
      numQuality: 0,
      screenshot_pub: null
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
    this.screenshot_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "screenshot",
      messageType: "std_msgs/UInt64",
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

    screenshot:function(id){
      var screenshotmsg = new ROSLIB.Message({"data" :id});
      this.screenshot_pub.publish(screenshotmsg);
    }
  },
};
</script>

<style scoped>
.box {
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
}
.rounded {
  border: 1px solid black;
  border-radius: 5px;
}

.button {
  height: 25px;
}
</style>
