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
        <p>Altitude: {{ odom.altitude.toFixed(2) }}m</p>
      </div>
      <div>
        <p>Odom Status: {{ get_odom_status }}</p>
      </div>
      <div>
        <p>Drone Status: {{ get_drone_status }}</p>
      </div>
    </div>
    <div class="calibration imu">
      <IMUCalibration></IMUCalibration>
    </div>
    <div class="flightindicator">
      <FlightAttitudeIndicator></FlightAttitudeIndicator>
    </div>
  </div>
</template>

<script lang="ts">
import { convertDMS } from '../utils.js'
import { mapGetters, mapState } from 'vuex'
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

  data() {
    return {
      drone_latitude_deg:42.293195,
      drone_longitude_deg:-83.7096706,
      drone_status: false
    }
  },
  computed: {
    ...mapState('websocket', ['message']),

    ...mapGetters('map', {
      odom_format: 'odomFormat'
    }),
    formatted_odom: function () {
      return {
        lat: convertDMS({ d: this.odom.latitude_deg, m: 0, s: 0 }, this.odom_format),
        lon: convertDMS({ d: this.odom.longitude_deg, m: 0, s: 0 }, this.odom_format)
      }
    },
    min_enabled: function () {
      return this.odom_format != 'D'
    },
    sec_enabled: function () {
      return this.odom_format == 'DMS'
    },
    alt_available: function () {
      return !isNan(this.odom.altitude)
    },
    get_odom_status: function () {
      if(this.odom.status){
        return "fixed"
      }
      else{
        return "not fixed"
      }
    },
    get_drone_status: function () {
      if(this.drone_status){
        return "fixed"
      }
      else{
        return "not fixed"
      }
    }
  },
  

  watch:{
      message(msg){
        if (msg.type == 'drone_waypoint') {
            this.drone_latitude_deg = msg.latitude
            this.drone_longitude_deg = msg.longitude
            this.drone_status = msg.status
          }
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
