<template>
  <div>
    <div v-show="false" id="key">
      <input @keydown="keyMonitorDown" />
      <input @keyup="keyMonitorUp" />
    </div>
    <input
      id="myRange"
      v-model="velocityScale"
      type="range"
      min="0"
      max="100"
    />Velocity Scaling: {{ velocityScale + "%" }}
  </div>
</template>

<script>
let interval;

export default {
  props: {
    forwardsKey:
    // ID of key to send positive velocity
    {
      type: Number,
      required: true,
    },

    backwardsKey:
    // ID of key to send negative velocity
    {
      type: Number,
      required: true,
    },

    updateRate:
    // In seconds
    {
      type: Number,
      required: false,
      default: 0.1,
    },

    scaleDefault:
    // Default velocityScale in percent
    {
      type: Number,
      required: false,
      default: 100,
    },
  },

  emits: ["velocity"],

  data() {
    return {
      velocity: 0,
      velocityScale: this.scaleDefault,
    };
  },

  computed: {
    velocityScaleDecimal: function () {
      return this.velocityScale / 100;
    },
  },

  created: function () {
    document.addEventListener("keyup", this.keyMonitorUp);
    document.addEventListener("keydown", this.keyMonitorDown);

    interval = setInterval(() => {
      this.$emit("velocity", this.velocity);
    }, this.updateRate * 1000);
  },

  beforeUnmount: function () {
    window.clearInterval(interval);
  },

  methods: {
    // when a key is being pressed down, sets input for that key as 1
    keyMonitorDown: function (event) {
      if (event.keyCode == this.forwardsKey) {
        this.velocity = this.velocityScaleDecimal;
      } else if (event.keyCode == this.backwardsKey) {
        this.velocity = -1 * this.velocityScaleDecimal;
      }
    },
    keyMonitorUp: function (event) {
      if (
        event.keyCode == this.forwardsKey ||
        event.keyCode == this.backwardsKey
      ) {
        this.velocity = 0;
      }
    },
  },
};
</script>

<style scoped>
#circle {
  height: 20px;
  width: 20px;
  border-radius: 50%;
  background-color: green;
  margin-right: 2%;
}

#box-1 {
  margin-top: 5%;
  display: flex;
  flex-direction: row;
}
</style>
