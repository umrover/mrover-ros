<template>
  <div class="box">
    <div class="identification">
      <p>{{ waypoint.name }}, ID: {{ waypoint.id }}</p>
    </div>
    <div class="buttons">
      <button
        class="button red"
        @click="$emit('add', { list: list, index: index })"
      >
        Add
      </button>
      <button
        class="button"
        :class="[waypoint.post ? 'green' : 'red']"
        @click="$emit('togglePost', { list: list, index: index })"
      >
        Post
      </button>
      <button
        class="button"
        :class="[waypoint.gate ? 'green' : 'red']"
        @click="$emit('toggleGate', { list: list, index: index })"
      >
        Gate
      </button>
      <button
        class="button red"
        @click="$emit('delete', { list: list, index: index })"
      >
        Delete
      </button>
      <button
        class="button"
        :class="[index === highlightedWaypoint ? 'green' : 'red']"
        @click="$emit('find', { list: list, index: index })"
      >
        Find
      </button>
    </div>
    <div class="location">
      <div>
        <p>{{ output.lat.d }}º</p>
        <p v-if="min_enabled">{{ output.lat.m }}'</p>
        <p v-if="sec_enabled">{{ output.lat.s }}"</p>
        N <b>&nbsp;|</b>
        <p>{{ output.lon.d }}º</p>
        <p v-if="min_enabled">{{ output.lon.m }}'</p>
        <p v-if="sec_enabled">{{ output.lon.s }}"</p>
        W
      </div>
    </div>
  </div>
</template>

<script>
import "../assets/style.css";
import { mapGetters } from "vuex";
import { convertDMS } from "../utils";

export default {
  props: {
    waypoint: {
      type: Object,
      required: true,
    },

    list: {
      type: Number,
      required: true,
    },

    index: {
      type: Number,
      required: true,
    },
  },

  computed: {
    ...mapGetters("autonomy", {
      odom_format: "odomFormat",
      highlightedWaypoint: "highlightedWaypoint",
    }),

    min_enabled: function () {
      return this.odom_format != "D";
    },

    sec_enabled: function () {
      return this.odom_format == "DMS";
    },

    output: function () {
      return {
        lat: convertDMS({ d: this.waypoint.lat, m: 0, s: 0 }, this.odom_format),
        lon: convertDMS({ d: this.waypoint.lon, m: 0, s: 0 }, this.odom_format),
      };
    },
  },
};
</script>

<style scoped>
.box {
  padding: 1px 10px 10px 10px;
  margin: 5px;
}
.location {
  grid-area: location;
}

.location p {
  display: inline;
}

.buttons {
  grid-area: buttons;
  text-align: center;
  display: block;
}

.button {
  width: 50px;
  height: auto;
  margin: 2px;
}

p {
  margin: 5px;
}
</style>
