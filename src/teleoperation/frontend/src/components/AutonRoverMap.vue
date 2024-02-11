<template>
  <div class="wrap">
    <!-- Leaflet Map Definition-->
    <l-map
      @ready="onMapReady"
      ref="map"
      class="map"
      :zoom="22"
      :center="center"
      @click="getClickedLatLon($event)"
    >
      <l-control-scale :imperial="false" />
      <!-- Tile Layer for map background -->
      <l-tile-layer
        ref="tileLayer"
        :url="online ? onlineUrl : offlineUrl"
        :attribution="attribution"
        :options="online ? onlineTileOptions : offlineTileOptions"
      />

      <!-- Markers for rover location -->
      <!-- TODO: Figure out if we still want these -->
      <l-marker ref="rover" :lat-lng="odomLatLng" :icon="locationIcon" />

      <!-- Waypoint Icons -->
      <l-marker
        v-for="(waypoint, index) in waypointList"
        :key="index"
        :lat-lng="waypoint.latLng"
        :icon="waypointIcon"
      >
        <l-tooltip :options="{ permanent: 'true', direction: 'top' }">
          {{ waypoint.name }}, {{ index }}
        </l-tooltip>
      </l-marker>

      <!-- Search Path Icons -->
      <l-marker
        v-for="(search_path_point, index) in searchPathPoints"
        :key="index"
        :lat-lng="search_path_point.latLng"
        :icon="searchPathIcon"
      >
        <l-tooltip>Search Path {{ index }}</l-tooltip>
      </l-marker>

      <!-- Polylines -->
      <l-polyline :lat-lngs="polylinePath" :color="'red'" :dash-array="'5, 5'" />
      <l-polyline :lat-lngs="odomPath" :color="'blue'" />
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
import { mapGetters, mapMutations } from 'vuex'
import 'leaflet/dist/leaflet.css'
import L from '../leaflet-rotatedmarker'

const MAX_ODOM_COUNT = 1000
const DRAW_FREQUENCY = 10
// Options for the tilelayer object on the map
const onlineUrl = 'http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'
const offlineUrl = 'map/{z}/{x}/{y}.png'
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
    }
  },
  data() {
    return {
      center: L.latLng(42.293195, -83.7096706),
      attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
      online: true,
      onlineUrl: onlineUrl,
      offlineUrl: offlineUrl,
      onlineTileOptions: onlineTileOptions,
      offlineTileOptions: offlineTileOptions,
      roverMarker: null,
      waypointIcon: null,
      searchPathIcon: null,
      gatePathIcon: null,

      map: null,
      odomCount: 0,
      locationIcon: null,
      odomPath: [],

      searchPathPoints: [],
      gatePathPoints: [],

      post1: null,
      post2: null,

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
        // console.log(this.roverMarker)
        this.roverMarker.setRotationAngle(angle)

        this.roverMarker.setLatLng(latLng)

        // Update the rover path
        this.odomCount++
        if (this.odomCount % DRAW_FREQUENCY === 0) {
          if (this.odomCount > MAX_ODOM_COUNT * DRAW_FREQUENCY) {
            this.odomPath.splice(0, 1)
          }
          this.odomPath.push(latLng)
        }

        this.odomPath[this.odomPath.length - 1] = latLng
      },
      // Deep will watch for changes in children of an object
      deep: true
    },
    autonEnabled: {
      handler: function () {
        if (this.autonEnabled) {
          this.searchPathPoints = []
          this.gatePathPoints = []

          this.post1 = null
          this.post2 = null
        }
      }
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
    this.searchPathIcon = L.icon({
      iconUrl: 'map_marker_projected.png',
      iconSize: [32, 32],
      iconAnchor: [16, 16],
      popupAnchor: [0, -32]
    })

    //   this.search_path_topic = new ROSLIB.Topic({
    //     ros: this.$ros,
    //     name: "/search_path",
    //     messageType: "mrover/GPSPointList"
    //   });

    //   this.search_path_topic.subscribe((msg) => {
    //     let newSearchPath = msg.point;
    //     this.searchPathPoints = newSearchPath.map((search_path_point) => {
    //       return {
    //         latLng: L.latLng(
    //           search_path_point.latitude_degrees,
    //           search_path_point.longitude_degrees
    //         )
    //       };
    //     });
    //   });
  },
  // Pull objects from refs to be able to access data and change w functions
  mounted: function () {},

  methods: {
    onMapReady: function (ready) {
      this.$nextTick(() => {
        this.map = this.$refs.map.leafletObject
        console.log(this.$refs.rover)
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
  height: 100%;
  display: grid;
  overflow: hidden;
  min-height: 100%;
  grid-gap: 3px;
  grid-template-columns: auto;
  grid-template-rows: 94% 6%;
  grid-template-areas:
    'map'
    'controls';
}

.custom-tooltip {
  display: inline-block;
  margin: 10px 20px;
  opacity: 1;
  position: relative;
}

.custom-tooltip .tooltip-inner {
  background: #0088cc;
}

.custom-tooltip.top .tooltip-arrow {
  border-top-color: #0088cc;
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
