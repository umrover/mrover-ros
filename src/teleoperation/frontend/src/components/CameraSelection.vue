<template>
  <div class="wrap">
    <div v-for="j in 3" :key="j" class="buttons">
      <template v-for="i in 3" :key="i">
        <button
          :class="['btn', color(i, j), 'mx-1', 'my-1']"
          @click="$emit('cam_index', i - 1 + 3 * (j - 1))"
        >
          <span>{{ names[i - 1 + 3 * (j - 1)] }}</span>
        </button>
      </template>
    </div>
  </div>
</template>

<script lang="ts">
export default {
  props: {
    capacity: {
      type: Number,
      required: true
    },
    camsEnabled: {
      type: Array,
      required: true
    },
    names: {
      type: Array,
      required: true
    }
  },
  data() {
    return {
      opacities: new Array(9).fill(1.0)
    }
  },

  methods: {
    color: function(i: number, j: number) {
      return this.camsEnabled[i - 1 + 3 * (j - 1)] ? 'btn-success' : 'btn-secondary'
    }
  },

  watch: {
    camsEnabled: function() {
      for (let i = 0; i < this.camsEnabled.length; i++) {
        this.camsEnabled[i] ? (this.opacities[i] = 0.55) : (this.opacities[i] = 1.0)
      }
    }
  }
}
</script>

<style scoped>
.buttons {
  display: flex;
  align-items: center;
  justify-content: center;
}
</style>
