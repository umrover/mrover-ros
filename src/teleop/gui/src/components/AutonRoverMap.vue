<template>
  <div class="wrap">
    <!-- Leaflet Map Definition-->
    <l-map
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
        :options="
          online ? onlineTileOptions : offlineTileOptions
        "
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

      <!-- Projected Icons -->
      <l-marker
        v-for="(projected_point, index) in projectedPoints"
        :key="index"
        :lat-lng="projected_point.latLng"
        :icon="projectedPointIcon"
      >
        <l-tooltip :options="{ permanent: 'true', direction: 'top' }">Projected {{
          index
        }}</l-tooltip>
      </l-marker>

      <!-- Gate Post Icons -->
      <l-marker v-if="post1" :lat-lng="post1" :icon="postIcon">
        <l-tooltip :options="{ permanent: 'true', direction: 'top' }"
          >Post 1</l-tooltip
        >
      </l-marker>
      <l-marker v-if="post2" :lat-lng="post2" :icon="postIcon">
        <l-tooltip :options="{ permanent: 'true', direction: 'top' }"
          >Post 2</l-tooltip
        >
      </l-marker>

      <!-- Polylines -->
      <l-polyline
        :lat-lngs="polylinePath"
        :color="'red'"
        :dash-array="'5, 5'"
      />
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

<script>
import ROSLIB from "roslib";
import {
  LMap,
  LTileLayer,
  LMarker,
  LPolyline,
  LPopup,
  LTooltip,
  LControlScale
} from "vue2-leaflet";
import { mapGetters, mapMutations } from "vuex";
import L from "../leaflet-rotatedmarker";

const MAX_ODOM_COUNT = 1000;
const DRAW_FREQUENCY = 10;
// Options for the tilelayer object on the map
const onlineUrl = "http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}";
const offlineUrl = "/static/map/{z}/{x}/{y}.png";
const onlineTileOptions = {
  maxNativeZoom: 22,
  maxZoom: 100,
  subdomains: ["mt0", "mt1", "mt2", "mt3"]
};
const offlineTileOptions = {
  maxNativeZoom: 22,
  maxZoom: 100
};

export default {
  name: "AutonRoverMap",

  components: {
    LMap,
    LTileLayer,
    LMarker,
    LPolyline,
    LPopup,
    LTooltip,
    LControlScale
  },

  created: function () {
    // Get Icons for Map
    this.locationIcon = L.icon({
      iconUrl: "/static/location_marker_icon.png",
      iconSize: [40, 40],
      iconAnchor: [20, 20]
    });
    this.waypointIcon = L.icon({
      iconUrl: "/static/map_marker.png",
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    });
    this.projectedPointIcon =
      L.icon({
        iconUrl: "/static/map_marker_projected.png",
        iconSize: [64, 64],
        iconAnchor: [32, 64],
        popupAnchor: [0, -32]
      });
    this.postIcon = L.icon({
      iconUrl: "/static/gate_location.png",
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    });

    this.projected_topic = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/projected_points",
      messageType: "mrover/NavMetadata"
    });

    this.gate_topic = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/estimated_gate_location",
      messageType: "mrover/NavMetadata"
    });

    this.projected_topic.subscribe((msg) => {
      let newProjectedList = msg.points;
      this.projectedPoints = newProjectedList.map((projected_point) => {
        return {
          latLng: L.latLng(
            //42.295,
            //-83.7076
            projected_point.latitude_degrees,
            projected_point.longitude_degrees
          )
        };
      });
    });

    this.gate_topic.subscribe((msg) => {
      this.post1 = L.latLng(
        msg.points[0].latitude_degrees,
        msg.points[0].longitude_degrees
      );
      this.post2 = L.latLng(
        msg.points[1].latitude_degrees,
        msg.points[1].longitude_degrees
      );
    });
  },

  computed: {
    ...mapGetters("autonomy", {
      route: "route",
      waypointList: "waypointList"
    }),

    // Convert to latLng object for Leaflet to use
    odomLatLng: function () {
      return L.latLng(this.odom.latitude_deg, this.odom.longitude_deg);
    },

    // Concat waypoints on course with rover marker at index 0 for polyline
    polylinePath: function () {
      return [this.odomLatLng].concat(
        this.route.map((waypoint) => waypoint.latLng)
      );
    }
  },

  props: {
    odom: {
      type: Object,
      required: true
    }
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
      projectedPointIcon: null,
      map: null,
      odomCount: 0,
      locationIcon: null,
      odomPath: [],

      projectedPoints: [],

      post1: null,
      post2: null,

      findRover: false
    };
  },

  methods: {
    // Event listener for setting store values to get data to waypoint Editor
    getClickedLatLon: function (e) {
      this.setClickPoint({
        lat: e.latlng.lat,
        lon: e.latlng.lng
      });
    },

    ...mapMutations("autonomy", {
      setClickPoint: "setClickPoint",
      setWaypointList: "setWaypointList",
      setOdomFormat: "setOdomFormat"
    })
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
      deep: true
    }
  },

  // Pull objects from refs to be able to access data and change w functions
  mounted: function () {
    this.$nextTick(() => {
      this.map = this.$refs.map.mapObject;
      this.roverMarker = this.$refs.rover.mapObject;
    });
  }
};
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
  grid-template-columns: 1fr;
  grid-template-rows: 94% 6%;
  grid-template-areas:
    "map"
    "controls";
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
  grid-area: "map";
}

.controls {
  grid-area: "controls";
  display: inline;
}
</style>
