<template>
  <button :class="['btn', color]" @click="toggleAndEmit()">
    <span class="white-text">{{ name }}: <br />{{ active ? '\u2611' : '\u2610' }}</span>
  </button>
</template>

<script lang="ts">
export default {
  props: {
    name: {
      type: String,
      required: true
    },
    color: {
      type: String,
      required: true
    }
  },
  data() {
    return {
      active: false
    }
  },

  methods: {
    toggle: function () {
      this.active = !this.active
    },

    toggleAndEmit: function () {
      this.toggle()
      this.$emit('toggle', this.active)
    }
  }
}
</script>

<style scoped>
.wrap-button {
  display: block;
  align-items: center;
  justify-content: center;
  padding: 1px;
}

.white-text {
  font-size: large;
  color: white;
}
</style>
