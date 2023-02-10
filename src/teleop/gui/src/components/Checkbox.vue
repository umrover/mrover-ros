<template>
  <button class="button" :class="[color]" @click="toggleAndEmit()">
    <span class="white-text"
      >{{ name }}: {{ active ? "\u2611" : "\u2610" }}</span
    >
  </button>
</template>

<script>
import '../assets/style.css';

export default {
  props: {
    name: {
      type: String,
      required: true,
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
.white-text {
  color: white;
}
</style>
