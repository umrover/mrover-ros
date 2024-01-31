<template>
  <div class="shadow my-1 p-3 rounded waypoint-item">
    <div class="identification">
      <p>{{ waypoint.name }} | ID: {{ waypoint.id }} | Type: {{ WAYPOINT_TYPES[waypoint.type] }}</p>
    </div>
    <div class="row">
      <div class="col text-center">
        <button class="btn btn-danger" @click="$emit('add', { in_route: in_route, index: index })">
          Add
        </button>
        <button
          class="btn btn-danger"
          @click="$emit('delete', { in_route: in_route, index: index })"
        >
          Delete
        </button>
      </div>
    </div>
    <div class="location">
      <p>{{ output.lat.d }}º</p>
      <p v-if="min_enabled">{{ output.lat.m }}'</p>
      <p v-if="sec_enabled">{{ output.lat.s }}"</p>
      N <b>|</b>
      <p>{{ output.lon.d }}º</p>
      <p v-if="min_enabled">{{ output.lon.m }}'</p>
      <p v-if="sec_enabled">{{ output.lon.s }}"</p>
      W
    </div>
  </div>
</template>

<script lang="ts">
import { mapGetters } from 'vuex'
import { convertDMS } from '../utils'

const WAYPOINT_TYPES = {
  0: 'No Search',
  1: 'Post',
  2: 'Mallet',
  3: 'Water Bottle'
}

export default {
  data() {
    return {
      WAYPOINT_TYPES: WAYPOINT_TYPES
    }
  },

  props: {
    waypoint: {
      type: Object,
      required: true
    },

    in_route: {
      type: Boolean,
      required: true
    },

    index: {
      type: Number,
      required: true
    }
  },

  computed: {
    ...mapGetters('map', {
      odom_format: 'odomFormat'
    }),

    ...mapGetters('autonomy', {
      highlightedWaypoint: 'highlightedWaypoint'
    }),

    min_enabled: function () {
      return this.odom_format != 'D'
    },

    sec_enabled: function () {
      return this.odom_format == 'DMS'
    },

    output: function () {
      return {
        lat: convertDMS({ d: this.waypoint.lat, m: 0, s: 0 }, this.odom_format),
        lon: convertDMS({ d: this.waypoint.lon, m: 0, s: 0 }, this.odom_format)
      }
    }
  }
}
</script>

<style scoped>
.location p {
  display: inline-block;
  margin: 2px;
}

button {
  margin: 0px 2px 0px 2px;
}
</style>
