<template>
  <div class="wrap">
    <l-map
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
        <div v-if="index === highlightedWaypoint">
          <l-marker :lat-lng="waypoint.latLng" :icon="highlightedWaypointIcon">
            <l-tooltip :options="{ permanent: 'true', direction: 'top' }">
              {{ waypoint.name }}, {{ index }}
            </l-tooltip>
          </l-marker>
        </div>
        <div v-else>
          <l-marker :lat-lng="waypoint.latLng" :icon="waypointIcon">
            <l-tooltip :options="{ permanent: 'true', direction: 'top' }">
              {{ waypoint.name }}, {{ index }}
            </l-tooltip>
          </l-marker>
        </div>
      </div>

      <l-polyline :lat-lngs="odomPath" :color="'blue'" />
    </l-map>
    <div class="controls">
      <div class="online">
        <label><input v-model="online" type="checkbox" />Online</label>
      </div>
    </div>
  </div>
</template>

<script>
import {
  LMap,
  LTileLayer,
  LMarker,
  LPolyline,
  LPopup,
  LTooltip,
  LControlScale,
} from "vue2-leaflet";
import { mapGetters, mapMutations } from "vuex";
import L from "../leaflet-rotatedmarker.js";
import ROSLIB from "roslib";

const MAX_ODOM_COUNT = 1000;
const DRAW_FREQUENCY = 10;
const onlineUrl = "http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}";
const offlineUrl = "/static/map/{z}/{x}/{y}.png";
const onlineTileOptions = {
  maxNativeZoom: 22,
  maxZoom: 100,
  subdomains: ["mt0", "mt1", "mt2", "mt3"],
};
const offlineTileOptions = {
  maxNativeZoom: 16,
  maxZoom: 100,
};

export default {
  name: "RoverMap",

  components: {
    LMap,
    LTileLayer,
    LMarker,
    LPolyline,
    LPopup,
    LTooltip,
    LControlScale,
  },

  data() {
    return {
      // Default Center In NC 53 Parking Lot
      center: L.latLng(42.294864932393835, -83.70781314674628),
      attribution:
        '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
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
      findRover: false,
    };
  },

  created: function () {
    this.locationIcon = L.icon({
      iconUrl: "/static/location_marker_icon.png",
      iconSize: [64, 64],
      iconAnchor: [32, 32],
    });
    this.waypointIcon = L.icon({
      iconUrl: "/static/map_marker.png",
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32],
    });
    this.highlightedWaypointIcon = L.icon({
      iconUrl: "/static/map_marker_highlighted.png",
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32],
    });
  },

  methods: {
    // Event listener for setting store values to get data to waypoint Editor
    getClickedLatLon: function (e) {
      this.setClickPoint({
        lat: e.latlng.lat,
        lon: e.latlng.lng,
      });
    },
    ...mapMutations("erd", {
      setClickPoint: "setClickPoint",
      setWaypointList: "setWaypointList",
      setOdomFormat: "setOdomFormat",
    }),
  },

  computed: {
    ...mapGetters("erd", {
      waypointList: "waypointList",
      highlightedWaypoint: "highlightedWaypoint",
    }),

    // Convert to latLng object for Leaflet to use
    odomLatLng: function () {
      return L.latLng(this.odom.latitude_deg, this.odom.longitude_deg);
    },

    polylinePath: function () {
      return [this.odomLatLng].concat(
        this.route.map((waypoint) => waypoint.latLng)
      );
    },
  },

  props: {
    odom: {
      type: Object,
      required: true,
    },
  },

  watch: {
    odom: {
      handler: function (val) {
        // Trigger every time rover odom is changed

        const lat = val.latitude_deg;
        const lng = val.longitude_deg;
        const angle = val.bearing_deg;

        const latLng = L.latLng(lat, lng);

        // Move to rover on first odom message
        if (!this.findRover) {
          this.findRover = true;
          this.center = latLng;
        }

        // Update the rover marker using bearing angle
        this.roverMarker.setRotationAngle(angle);

        this.roverMarker.setLatLng(latLng);

        // Update the rover path
        this.odomCount++;
        if (this.odomCount % DRAW_FREQUENCY === 0) {
          if (this.odomCount > MAX_ODOM_COUNT * DRAW_FREQUENCY) {
            this.odomPath.splice(0, 1);
          }
          this.odomPath.push(latLng);
        }

        this.odomPath[this.odomPath.length - 1] = latLng;
      },
      // Deep will watch for changes in children of an object
      deep: true,
    },
  },
  mounted: function () {
    this.$nextTick(() => {
      this.map = this.$refs.map.mapObject;
      this.roverMarker = this.$refs.rover.mapObject;
    });
  },
};
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
}
</style>
