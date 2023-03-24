<template>
  <div class="wrap">
    <span :class="['led', color]"></span>
    <span class="name">{{ name }}</span>
  </div>
</template>

<script>
export default {
  props: {
    connected: {
      type: Boolean,
      required: true,
    },
    name: {
      type: String,
      required: true,
    },
  },
  computed: {
    color: function () {
      return this.connected ? "green" : "red";
    },
  },
};
</script>

<style scoped>
.wrap {
  display: flex;
  align-items: center;
  height: 100%;
}
.led {
  width: 24px;
  height: 24px;
  border-radius: 12px;
}
.name {
  margin-left: 5px;
}
</style>
