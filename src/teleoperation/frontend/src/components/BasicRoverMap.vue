<template>
  <div class="wrap">
    <l-map
      @ready="onMapReady"
      ref="map"
      class="map"
      :zoom="22"
      :center="center"
      @click="getClickedLatLon($event)"
    >
      <l-control-scale :imperial="false" />
      <l-tile-layer
        ref="tileLayer"
        :url="online ? onlineUrl : offlineUrl"
        :attribution="attribution"
        :options="online ? onlineTileOptions : offlineTileOptions"
      />

      <l-marker ref="rover" :lat-lng="odomLatLng" :icon="locationIcon" />

      <div v-for="(waypoint, index) in waypointList" :key="index">
        <l-marker :lat-lng="waypoint.latLng" :icon="getWaypointIcon(waypoint.drone)">
          <l-tooltip :options="{ permanent: 'true', direction: 'top' }">
            {{ waypoint.name }}, {{ index }}
          </l-tooltip>
        </l-marker>
      </div>

      <l-polyline :lat-lngs="odomPath" :color="'blue'" />
    </l-map>
    <label><input v-model="online" type="checkbox" />Online</label>
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
import { mapGetters, mapMutations } from 'vuex'
import 'leaflet/dist/leaflet.css'
import L from '../leaflet-rotatedmarker.js'

const MAX_ODOM_COUNT = 1000
const DRAW_FREQUENCY = 10
const onlineUrl = 'http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'
const offlineUrl = 'map/urc/{z}/{x}/{y}.jpg'
const onlineTileOptions = {
  maxNativeZoom: 22,
  maxZoom: 100,
  subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
}
const offlineTileOptions = {
  maxNativeZoom: 16,
  maxZoom: 100
}

export default {
  name: 'RoverMap',

  components: {
    LMap,
    LTileLayer,
    LMarker,
    LPolyline,
    LTooltip,
    LControlScale
  },

  data() {
    return {
      // Default Center at MDRS
      center: L.latLng(38.406025, -110.7923723),
      attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
      online: true,
      onlineUrl: onlineUrl,
      offlineUrl: offlineUrl,
      onlineTileOptions: onlineTileOptions,
      offlineTileOptions: offlineTileOptions,
      roverMarker: null,
      waypointIcon: null,
      highlightedWaypointIcon: null,
      map: null,
      odomCount: 0,
      locationIcon: null,
      odomPath: [],
      findRover: false
    }
  },

  created: function() {
    this.locationIcon = L.icon({
      iconUrl: '/location_marker_icon.png',
      iconSize: [64, 64],
      iconAnchor: [32, 32]
    })
    this.waypointIcon = L.icon({
      iconUrl: '/map_marker.png',
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    })
    this.droneWaypointIcon = L.icon({
      iconUrl: '/map_marker_drone.png',
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    })
    this.highlightedWaypointIcon = L.icon({
      iconUrl: '/map_marker_highlighted.png',
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    })
  },

  methods: {
    onMapReady: function() {
      // Pull objects from refs to be able to access data and change w functions
      this.$nextTick(() => {
        this.map = this.$refs.map.leafletObject
        this.roverMarker = this.$refs.rover.leafletObject
      })
    },
    // Event listener for setting store values to get data to waypoint Editor
    getClickedLatLon: function(e: any) {
      this.setClickPoint({
        lat: e.latlng.lat,
        lon: e.latlng.lng
      })
    },
    getWaypointIcon: function(isDrone: boolean) {
      if (isDrone) {
        return this.droneWaypointIcon
      } else {
        return this.waypointIcon
      }
    },
    ...mapMutations('erd', {
      setClickPoint: 'setClickPoint',
      setWaypointList: 'setWaypointList',
      setOdomFormat: 'setOdomFormat'
    })
  },

  computed: {
    ...mapGetters('erd', {
      waypointList: 'waypointList',
      highlightedWaypoint: 'highlightedWaypoint'
    }),

    // Convert to latLng object for Leaflet to use
    odomLatLng: function() {
      return L.latLng(this.odom.latitude_deg, this.odom.longitude_deg)
    },

    polylinePath: function() {
      return [this.odomLatLng].concat(this.route.map((waypoint: any) => waypoint.latLng))
    }
  },

  props: {
    odom: {
      type: Object,
      required: true
    }
  },

  watch: {
    odom: {
      handler: function(val) {
        // Trigger every time rover odom is changed

        const lat = val.latitude_deg
        const lng = val.longitude_deg
        const angle = val.bearing_deg

        const latLng = L.latLng(lat, lng)

        // Move to rover on first odom message
        if (!this.findRover && this.map) {
          this.findRover = true
          this.map.setView(latLng, 22)
        }

        // Update the rover marker using bearing angle
        if (this.roverMarker) {
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
  }
}
</script>

<style scoped>
.map {
  height: 100%;
  width: 100%;
}

.wrap {
  display: flex;
  align-items: center;
  height: 100%;
  width: 100%;
}
</style>
