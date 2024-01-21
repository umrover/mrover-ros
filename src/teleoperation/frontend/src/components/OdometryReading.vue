<template>
  <div class="odom-wrap">
    <div class="odom">
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
      <div>
        <p v-if="alt_available">Altitude: {{ odom.altitude.toFixed(2) }}m</p>
      </div>
    </div>
    <!-- <div class="calibration imu">
      <IMUCalibration></IMUCalibration>
    </div>
    <div class = "flightindicator">
      <FlightAttitudeIndicator></FlightAttitudeIndicator>
    </div> -->
  </div>
</template>

<script lang="ts">
<<<<<<< HEAD
import { convertDMS } from "../utils.js";
import { mapGetters } from "vuex";
=======
import { convertDMS } from '../utils.js'
import { mapGetters } from 'vuex'
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
// import IMUCalibration from "./IMUCalibration.vue";
// import FlightAttitudeIndicator from "./FlightAttitudeIndicator.vue";
export default {
  components: {
    // FlightAttitudeIndicator,
    // IMUCalibration,
  },
  props: {
    odom: {
      type: Object,
      required: true
    }
  },

  computed: {
    ...mapGetters('map', {
      odom_format: 'odomFormat'
    }),
    formatted_odom: function () {
      return {
<<<<<<< HEAD
        lat: convertDMS(
          { d: this.odom.latitude_deg, m: 0, s: 0 },
          this.odom_format
        ),
        lon: convertDMS(
          { d: this.odom.longitude_deg, m: 0, s: 0 },
          this.odom_format
        ),
      };
=======
        lat: convertDMS({ d: this.odom.latitude_deg, m: 0, s: 0 }, this.odom_format),
        lon: convertDMS({ d: this.odom.longitude_deg, m: 0, s: 0 }, this.odom_format)
      }
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
    },
    min_enabled: function () {
      return this.odom_format != 'D'
    },
    sec_enabled: function () {
<<<<<<< HEAD
      return this.odom_format == "DMS";
    },
    alt_available: function () {
      // return isNan(this.odom.altitude);
      return false;
=======
      return this.odom_format == 'DMS'
    },
    alt_available: function () {
      // return isNan(this.odom.altitude);
      return false
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
    }
  }
}
</script>

<style scoped>
.odom-wrap {
  padding: 0px;
  padding-left: 10px;
  padding-right: 0px;
  border: none;
  margin-top: 0.5rem;
  display: grid;
  grid-gap: 10px;
  grid-template-columns: 300px 2fr 1.25fr 0.75fr;
  grid-template-rows: 140px 2fr;
  flex-direction: row;
  gap: 10px;
  grid-template-areas:
    'odom flightIndicator'
    'imu flightIndicator';
  height: auto;
  width: auto;
}
.odom-wrap p {
  display: inline;
}

.calibration {
  border: 1px solid black;
  padding-left: 10px;
  margin-bottom: 5px;
  margin-top: 5px;
  background-color: rgb(180, 180, 180);
}

.odom {
  grid-area: odom;
}

.flightIndicator {
  grid-area: flightIndicator;
}

.imu {
  grid-area: imu;
}
<<<<<<< HEAD


</style>
=======
</style>
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
