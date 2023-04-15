<template>
  <div class="box waypoint-item">
    <div class="name">
      <p>{{ waypoint.name }}</p>
    </div>
    <div class="location">
      <p>{{ waypoint.lat }}ºN, {{ waypoint.lon }}ºE</p>
    </div>
    <div class="buttons">
      <button class="button red" @click="$emit('delete', { index: index })">
        X
      </button>
      <button
        class="button"
        :class="[index === highlightedWaypoint ? 'green' : 'red']"
        @click="$emit('find', { index: index })"
      >
        Find
      </button>
    </div>
  </div>
</template>

<script>
import "../assets/style.css";
import { mapGetters } from "vuex";

export default {
  props: {
    waypoint: {
      type: Object,
      required: true,
    },
    index: {
      type: Number,
      required: true,
    },
  },

  computed: {
    ...mapGetters("erd", {
      highlightedWaypoint: "highlightedWaypoint",
    }),
  },
};
</script>

<style scoped>
.waypoint-item {
  display: grid;
  grid-template-columns: 2fr 1fr;
  grid-template-rows: 1fr 1fr;
  grid-template-areas: "name buttons" "location buttons";
  padding: 10px;
  margin: 5px;
}

.waypoint-item > div {
  margin: 5px;
}

.name {
  grid-area: name;
}

.location {
  grid-area: location;
}

.buttons {
  grid-area: buttons;
  align-self: center;
  justify-self: center;
}

.button {
  width: auto;
  height: auto;
  margin: 2px;
  padding: 7px;
  font-weight: bold;
}

p {
  margin: 0px;
}
</style>
