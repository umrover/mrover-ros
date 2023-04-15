<template>
  <div class="wrap">
    <div v-for="j in 3" :key="j" class="buttons">
        <button
          v-for="i in 3"
          :key="i"
          class="button"
          :class="{ active_cam_button: camsEnabled[j - 1 + 3 * (i - 1)] }"
          :disabled="maxedOut && !camsEnabled[j - 1 + 3 * (i - 1)]"
          @click="$emit('cam_index', j - 1 + 3 * (i - 1))"
        >
          <span>{{ names[j - 1 + 3 * (i - 1)] }}</span>
        </button>
    </div>
  </div>
</template>

<script>
import "../assets/style.css";

export default {
  props: {
    capacity: {
      type: Number,
      required: true,
    },
    camsEnabled: {
      type: Array,
      required: true,
    },
    names: {
      type: Array,
      required: true,
    },
  },
  data() {
    return {
      opacities: new Array(9).fill(1.0),
    };
  },

  computed: {
    maxedOut() {
      let num_enabled = 0;
      for (let i = 0; i < this.camsEnabled.length; i++) {
        if (this.camsEnabled[i]) {
          num_enabled++;
        }
      }
      return num_enabled == this.capacity;
    },
  },

  watch: {
    camsEnabled: function () {
      for (let i = 0; i < this.camsEnabled.length; i++) {
        this.camsEnabled[i]
          ? (this.opacities[i] = 0.55)
          : (this.opacities[i] = 1.0);
      }
    },
  },
};
</script>

<style scoped>
.buttons {
  display: block;
  margin: 5px;
}
.button {
  margin: 5px;
  width: 100%;
}

.active_cam_button {
  background-color: var(--accent);
}
</style>
