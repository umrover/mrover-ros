<template>
  <button v-if="!disabled" class="button" :class="[color]" @click="toggleAndEmit()">
    <span class="white-text"
      >{{ name }}: {{ active ? "\u2611" : "\u2610" }}</span
    >
  </button>
  <button v-else class="button button-disabled" :class="[color]">
    <span class="white-text"
      >{{ name }}: {{ active ? "\u2611" : "\u2610" }}</span
    >
  </button>
</template>

<script>
import "../assets/style.css";

export default {
  props: {
    name: {
      type: String,
      required: true,
    },
    disabled: {
      type: Boolean,
      default: false,
      required: false,
    },
  },
  data() {
    return {
      active: false,
    };
  },

  computed: {
    color: function () {
      return this.active ? "green" : "red";
    },
  },

  methods: {
    toggle: function () {
      this.active = !this.active;
    },

    toggleAndEmit: function () {
      this.toggle();
      this.$emit("toggle", this.active);
    },
  },
};
</script>

<style scoped>
.button-disabled {
  opacity: 0.5;
}

.white-text {
  color: white;
}
</style>
