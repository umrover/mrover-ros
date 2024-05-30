<template>
  <div class="wrap">
    <!-- Leaflet Map Definition-->
    <l-map @ready="onMapReady" ref="map" class="map" :zoom="16" :center="center" @click="getClickedLatLon($event)">
      <l-control-scale :imperial="false" />
      <!-- Tile Layer for map background -->
      <l-tile-layer ref="tileLayer" :url="online ? onlineUrl : offlineUrl" :attribution="attribution"
        :options="online ? onlineTileOptions : offlineTileOptions" />

      <!-- Markers for rover location -->
      <!-- TODO: Figure out if we still want these -->
      <l-marker ref="rover" :lat-lng="odomLatLng" :icon="locationIcon" />

      <!-- Waypoint Icons -->
      <l-marker v-for="(waypoint, index) in waypointList" :key="index" :lat-lng="waypoint.latLng" :icon="waypointIcon">
        <l-tooltip :options="{ permanent: 'true', direction: 'top' }">
          {{ waypoint.name }}
        </l-tooltip>
      </l-marker>

      <!-- Polylines -->
      <l-polyline :lat-lngs="polylinePath" :color="'red'" :dash-array="'5, 5'" />
      <l-polyline :lat-lngs="odomPath" :color="'blue'" :dash-array="'5, 5'" />
    </l-map>
    <!-- Controls that go directly under the map -->
    <div class="controls">
      <div class="online">
        <label><input v-model="online" type="checkbox" />Online</label>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import {
  LMap,
  LTileLayer,
  LMarker,
  LPolyline,
  LTooltip,
  LControlScale
} from '@vue-leaflet/vue-leaflet'
import { mapGetters, mapMutations, mapActions } from 'vuex'
import 'leaflet/dist/leaflet.css'
import L from '../leaflet-rotatedmarker'

const MAX_ODOM_COUNT = 10
const DRAW_FREQUENCY = 10
// Options for the tilelayer object on the map
const onlineUrl = 'http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'
const offlineUrl = 'map/urc/{z}/{x}/{y}.jpg'
const onlineTileOptions = {
  maxNativeZoom: 22,
  maxZoom: 100,
  subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
}
const offlineTileOptions = {
  minZoom: 16,
  maxZoom: 20,
}

export default {
  name: 'AutonRoverMap',
  components: {
    LMap,
    LTileLayer,
    LMarker,
    LPolyline,
    LTooltip,
    LControlScale
  },
  props: {
    odom: {
      type: Object,
      required: true
    },
  },
  data() {
    return {
      center: L.latLng(38.4071654, -110.7923927),
      attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
      online: true,
      onlineUrl: onlineUrl,
      offlineUrl: offlineUrl,
      onlineTileOptions: onlineTileOptions,
      offlineTileOptions: offlineTileOptions,
      roverMarker: null,
      waypointIcon: null,
      locationIcon: null,

      map: null,
      odomCount: 0,
      odomPath: [],

      findRover: false
    }
  },
  computed: {
    ...mapGetters('autonomy', {
      route: 'route',
      waypointList: 'waypointList',
      autonEnabled: 'autonEnabled'
    }),

    // Convert to latLng object for Leaflet to use
    odomLatLng: function () {
      return L.latLng(this.odom.latitude_deg, this.odom.longitude_deg)
    },

    // Concat waypoints on course with rover marker at index 0 for polyline
    polylinePath: function () {
      return [this.odomLatLng].concat(
        this.route.map((waypoint: { latLng: any }) => waypoint.latLng)
      )
    }
  },
  watch: {
    odom: {
      handler: function (val) {
        // Trigger every time rover odom is changed
        const lat = val.latitude_deg
        const lng = val.longitude_deg
        const angle = val.bearing_deg

        const latLng = L.latLng(lat, lng)

        // Move to rover on first odom message
        if (!this.findRover) {
          this.findRover = true
          this.center = latLng
        }

        // Update the rover marker using bearing angle
        if (this.roverMarker !== null) {
          this.roverMarker.setRotationAngle(angle)
          this.roverMarker.setLatLng(latLng)
        }

        // Update the rover path
        this.odomCount++
        if (this.odomCount % DRAW_FREQUENCY === 0) {
          if (this.odomPath.length > MAX_ODOM_COUNT) {
            this.odomPath = [...this.odomPath.slice(1), latLng] //remove oldest element
          }

          this.odomPath = [...this.odomPath, latLng]
          this.odomCount = 0
        }
      },
      // Deep will watch for changes in children of an object
      deep: true
    }
  },
  created: function () {
    // Get Icons for Map
    this.locationIcon = L.icon({
      iconUrl: 'location_marker_icon.png',
      iconSize: [40, 40],
      iconAnchor: [20, 20]
    })
    this.waypointIcon = L.icon({
      iconUrl: 'map_marker.png',
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    })
  },

  methods: {
    onMapReady: function () {
      // Pull objects from refs to be able to access data and change w functions
      this.$nextTick(() => {
        this.map = this.$refs.map.leafletObject
        this.roverMarker = this.$refs.rover.leafletObject
      })
    },
    // Event listener for setting store values to get data to waypoint Editor
    getClickedLatLon: function (e: { latlng: { lat: any; lng: any } }) {
      this.setClickPoint({
        lat: e.latlng.lat,
        lon: e.latlng.lng
      })
    },

    ...mapMutations('autonomy', {
      setClickPoint: 'setClickPoint',
      setWaypointList: 'setWaypointList',
      setOdomFormat: 'setOdomFormat'
    })
  }
}
</script>

<style scoped>
.controls label {
  font-size: 12px;
}

.controls div {
  display: inline-block;
}

.online {
  float: right;
}

.wrap {
  align-items: center;
  width: 100%;
  height: 100%;
  display: grid;
  overflow: hidden;
  min-height: 40vh;
  grid-gap: 3px;
  grid-template-columns: auto;
  grid-template-rows: 94% 6%;
  grid-template-areas:
    'map'
    'controls';
}

/* Grid area declarations */
.map {
  height: 100%;
  width: 100%;
  grid-area: map;
}

.controls {
  grid-area: controls;
  display: inline;
}
</style>
