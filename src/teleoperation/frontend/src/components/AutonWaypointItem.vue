<template>
  <div class="shadow my-1 p-3 rounded waypoint-item">
    <div class="identification">
      <p>{{ waypoint.name }} | ID: {{ waypoint.id }}</p>
    </div>
    <div class="row">
      <div class="col text-center">
        <button
          class="btn btn-danger"
          @click="$emit('delete', { waypoint })"
        >
          Delete
        </button>
      </div>
    </div>
    <div class="location">
      <p>{{ waypoint.lat }}º</p>
      N <b>|</b>
      <p>{{ waypoint.lon }}º</p>
      W
    </div>
  </div>
</template>

<script lang="ts">
import { mapGetters } from 'vuex'
import { convertDMS } from '../utils'

export default {
  props: {
    waypoint: {
      type: Object,
      required: true
    },
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
