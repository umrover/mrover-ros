<template>
  <div
    class="shadow my-1 p-3 rounded waypoint-item"
    v-bind:class="{ 'drone-waypoint': waypoint.drone }"
  >
    <div class="name">
      <p>{{ waypoint.name }}</p>
    </div>
    <div class="location">
      <p>{{ waypoint.lat }}ºN, {{ waypoint.lon }}ºW</p>
    </div>
    <div class="text-center">
      <button class="btn btn-danger" @click="$emit('delete', { index: index })">X</button>
      <button
        :class="['btn', index === highlightedWaypoint ? 'btn-success' : 'btn-danger']"
        @click="$emit('find', { index: index })"
      >
        Find
      </button>
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import { mapGetters } from 'vuex'

export default defineComponent({
  props: {
    waypoint: {
      type: Object,
      required: true
    },
    index: {
      type: Number,
      required: true
    }
  },

  computed: {
    ...mapGetters('erd', {
      highlightedWaypoint: 'highlightedWaypoint'
    })
  }
})
</script>

<style scoped>
button {
  margin: 0px 2px 0px 2px;
}

.drone-waypoint {
  background-color: limegreen;
}
</style>
