<template>
  <div class="d-flex flex-row align-items-center">
    <span :class="['led', color]"></span>
    <span class="mx-1" v-if="show_name">{{ name }}</span>
  </div>
</template>

<script lang="ts">
export default {
  props: {
    connected: {
      type: Boolean,
      required: true
    },
    name: {
      type: String,
      required: false
    },
    show_name: {
      type: Boolean,
      required: false
    }
  },
  computed: {
    color: function () {
      return this.connected ? 'bg-success' : 'bg-danger'
    }
  }
}
</script>

<style scoped>
.wrap {
  display: flex;
  align-items: center;
  height: 100%;
}
.led {
  width: 16px;
  height: 16px;
  border-radius: 8px;
  border: 1px solid;
}
</style>
