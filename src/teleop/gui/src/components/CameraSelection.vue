<template>
  <div class="wrap">
    <div v-for="j in 3" :key="j" class="buttons">
      <template v-for="i in 3">
        <button
          v-if="i - 1 + 3 * (j - 1) < cams.length"
          class="cam_buttons"
          :class="{ active_cam_button: cams[i - 1 + 3 * (j - 1)].enabled }"
          :disabled="maxedOut && !cams[i - 1 + 3 * (j - 1)].enabled"
          @click="$emit('cam_index', indices[i - 1 + 3 * (j - 1)])"
        >
          <span>{{ cams[i - 1 + 3 * (j - 1)].name }}</span>
        </button>
        <div class="fixed-spacer"></div>
      </template>
    </div>
  </div>
</template>

<script>
export default {
  props: {
    capacity: {
      type: Number,
      required: true,
    },
    cams : {
      type: Array,
      required: true
    }
  },
  data() {
    return {
      opacities: new Array(9).fill(1.0),
      indices: [] //the values are the camera IDs, the indicies are the button indices
    };
  },

  computed: {
    maxedOut() {
      let num_enabled = 0;
      this.cams.forEach((c) => {
        if (c.enabled) {
          num_enabled++;
        }
      });
      return num_enabled == this.capacity;
    },
  },

  watch: {
    cams: function () {
      this.indices = [];  //if cam's size changes, add new ones to array/remove old ones
      this.cams.forEach((c) => {
        this.indices.push(c);
        c.enabled
          ? (this.opacities[c.index] = 0.55)
          : (this.opacities[c.index] = 1.0);
      });
    }
  },

  created: function() {
      this.cams.forEach((c) => {
        this.indices.push(c.index);
      });
  },
};
</script>

<style scoped>
.buttons {
  display: flex;
  align-items: center;
  justify-content: center;
  margin: 10px;
}
.cam_buttons {
  height: 25px;
  width: 100px;
  border: 1px solid black;
  border-radius: 5px;
}
.fixed-spacer {
  width: 10px;
  height: auto;
}

.active_cam_button {
  background-color: green;
  color: white;
}
</style>
