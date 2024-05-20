<template>
  <div class='odom-wrap'>
    <div class='odom'>
      <p>Current Odometry Reading:</p>
      <div>
        <p>{{ formatted_odom.lat.d }}º</p>
        <p v-if='min_enabled'>{{ formatted_odom.lat.m }}'</p>
        <p v-if='sec_enabled'>{{ formatted_odom.lat.s }}"</p>
        N
      </div>
      <div>
        <p>{{ formatted_odom.lon.d }}º</p>
        <p v-if='min_enabled'>{{ formatted_odom.lon.m }}'</p>
        <p v-if='sec_enabled'>{{ formatted_odom.lon.s }}"</p>
        E
        <br />
        <p>Bearing: {{ odom.bearing_deg.toFixed(2) }}º</p>
      </div>
      <div>
        <p>Altitude: {{ odom.altitude.toFixed(2) }}m</p>
      </div>
    </div>
    <div class='calibration imu'>
      <IMUCalibration />
    </div>
    <div class='flightindicator'>
      <FlightAttitudeIndicator />
    </div>
  </div>
</template>

<script lang='ts'>
import { convertDMS } from '../utils'
import { mapGetters } from 'vuex'
import IMUCalibration from './IMUCalibration.vue'
import FlightAttitudeIndicator from './FlightAttitudeIndicator.vue'

export default {
  components: {
    FlightAttitudeIndicator,
    IMUCalibration
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
    formatted_odom: function() {
      return {
        lat: convertDMS({ d: this.odom.latitude_deg, m: 0, s: 0 }, this.odom_format),
        lon: convertDMS({ d: this.odom.longitude_deg, m: 0, s: 0 }, this.odom_format)
      }
    },
    min_enabled: function() {
      return this.odom_format != 'D'
    },
    sec_enabled: function() {
      return this.odom_format == 'DMS'
    },
    alt_available: function() {
      return !isNan(this.odom.altitude)
    }
  }
}
</script>

<style scoped>
.odom-wrap {
  padding-left: 10px;
  padding-right: 0px;
  margin-top: 0.5rem;
  display: grid;
  grid-gap: 10px;
  grid-template-columns: auto auto;
  grid-template-rows: auto auto;
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

.odom {
  grid-area: odom;
}

.flightIndicator {
  grid-area: flightIndicator;
}

.imu {
  grid-area: imu;
}
</style>
