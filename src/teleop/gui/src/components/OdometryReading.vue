<template>
  <div class="odom-wrap">
    <p>Current odometry reading:</p>
    <div>
      <p>{{ formatted_odom.lat.d }}º</p>
      <p v-if="min_enabled">{{ formatted_odom.lat.m }}'</p>
      <p v-if="sec_enabled">{{ formatted_odom.lat.s }}"</p>
      N
    </div>
    <div>
      <p>{{ formatted_odom.lon.d }}º</p>
      <p v-if="min_enabled">{{ formatted_odom.lon.m }}'</p>
      <p v-if="sec_enabled">{{ formatted_odom.lon.s }}"</p>
      E
      <br />
      <p>Bearing: {{ odom.bearing_deg.toFixed(2) }}º</p>
    </div>
  </div>
</template>

<script>
import "../assets/style.css";
import { convertDMS } from "../utils.js";
import { mapGetters } from "vuex";
export default {
  props: {
    odom: {
      type: Object,
      required: true,
    },
  },

  computed: {
    ...mapGetters("map", {
      odom_format: "odomFormat",
    }),
    formatted_odom: function () {
      return {
        lat: convertDMS(
          { d: this.odom.latitude_deg, m: 0, s: 0 },
          this.odom_format
        ),
        lon: convertDMS(
          { d: this.odom.longitude_deg, m: 0, s: 0 },
          this.odom_format
        ),
      };
    },
    min_enabled: function () {
      return this.odom_format != "D";
    },
    sec_enabled: function () {
      return this.odom_format == "DMS";
    },
  },
};
</script>

<style scoped>
.odom-wrap {
  padding: 0px;
  padding-left: 10px;
  padding-right: 0px;
  border: none;
  margin-top: 0.5rem;
}
.odom-wrap p {
  display: inline;
}
</style>
