<template>
  <div class="wrap">
    <!-- Leaflet Map Definition-->
    <l-map ref="map" class="map" :zoom="15" :center="center" v-on:click="getClickedLatLon($event)">
      <l-control-scale :imperial="false"/>
      <!-- Tile Layer for map background -->
      <l-tile-layer :url="this.online ? this.onlineUrl : this.offlineUrl" :attribution="attribution" :options="tileLayerOptions"/>
      
      <!-- Markers for rover location -->
      <!-- TODO: Figure out if we still want these -->
      <!-- <l-marker ref="tangent" :lat-lng="odomLatLng" :icon="tangentIcon"/>
      <l-marker ref="target_bearing" :lat-lng="odomLatLng" :icon="targetBearingIcon"/> -->
      <l-marker ref="rover" :lat-lng="odomLatLng" :icon="locationIcon"/>

      <!-- Waypoint Icons -->
      <l-marker :lat-lng="waypoint.latLng" :icon="waypointIcon" v-for="(waypoint, index) in waypointList" :key="index">
         <l-tooltip :options="{permanent: 'true', direction: 'top'}"> {{ waypoint.name }}, {{ index }} </l-tooltip>
      </l-marker>

      <!-- Projected Point Icons -->
      <!-- <l-marker :lat-lng="projected_point.latLng" :icon="projectedPointIcon" v-for="(projected_point, index) in projectedPoints" :key="index">
        <l-tooltip :options="{permanent: 'true', direction: 'top'}">{{ projectedPointsType }} {{ index }}</l-tooltip>
      </l-marker> -->
      
      <!-- Gate Post Icons -->
      <l-marker :lat-lng="post1" :icon="postIcon" v-if="post1">
         <l-tooltip :options="{permanent: 'true', direction: 'top'}">Post 1</l-tooltip>
      </l-marker>
      <l-marker :lat-lng="post2" :icon="postIcon" v-if="post2">
         <l-tooltip :options="{permanent: 'true', direction: 'top'}">Post 2</l-tooltip>
      </l-marker>

      <!-- Polylines -->
      <l-polyline :lat-lngs="polylinePath" :color="'red'" :dash-array="'5, 5'"/>
      <l-polyline :lat-lngs="projectedPath" :color="'black'" :dash-array="'5, 5'" :fill="false"/>
      <l-polyline :lat-lngs="odomPath" :color="'blue'"/>
    </l-map>
    <!-- Controls that go directly under the map -->
    <div class="controls">
      <div class="online">
        <label><input type="checkbox" v-model="online" />Online</label>
      </div> 
    </div>
  </div>
</template>

<script>
import { LMap, LTileLayer, LMarker, LPolyline, LPopup, LTooltip, LControlScale } from 'vue2-leaflet'
import { mapGetters, mapMutations } from 'vuex'
import L from '../leaflet-rotatedmarker'

const MAX_ODOM_COUNT = 1000
const DRAW_FREQUENCY = 10
const onlineUrl = 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png'
const offlineUrl = '/static/map/{z}/{x}/{y}.png'

export default {
  name: 'AutonRoverMap',

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
      iconUrl: '/static/location_marker_icon.png',
      iconSize: [40, 40],
      iconAnchor: [20, 20]
    })
    this.tangentIcon = L.icon({
      iconUrl: '/static/gps_tangent_icon.png',
      iconSize: [44, 80],
      iconAnchor: [22, 60]
    })
    this.targetBearingIcon = L.icon({
      iconUrl: '/static/gps_tangent_icon.png',
      iconSize: [30, 56],
      iconAnchor: [15, 42]
    })
    this.waypointIcon = L.icon({
      iconUrl: '/static/map_marker.png',
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    })
    this.projectedPointIcon = L.icon({
      iconUrl: '/static/map_marker_projected.png',
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    })
    this.postIcon = L.icon({
      iconUrl: '/static/gate_location.png',
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    })

    // TODO: Remove these and replace with ROS topics if still needed
    //
    // this.$parent.subscribe('/projected_points', (msg) => {
    //   let newProjectedList = msg.points
    //   this.projectedPoints = newProjectedList.map((projected_point) => {
    //     return {
    //       latLng: L.latLng(
    //         projected_point.latitude_deg + projected_point.latitude_min/60,
    //         projected_point.longitude_deg + projected_point.longitude_min/60
    //       )
    //     }
    //   })

    //   this.projectedPointsType = msg.path_type
    // })

    // this.$parent.subscribe('/estimated_gate_location', (msg) => {
    //   this.post1 =  L.latLng(
    //     msg.post1.latitude_deg + msg.post1.latitude_min/60,
    //     msg.post1.longitude_deg + msg.post1.longitude_min/60
    //   )
    //   this.post2 = L.latLng(
    //     msg.post2.latitude_deg + msg.post2.latitude_min/60,
    //     msg.post2.longitude_deg + msg.post2.longitude_min/60
    //   )
    // })

  },

  computed: {
    ...mapGetters('autonomy', {
      route: 'route',
      waypointList: 'waypointList',
    }),

    // Convert to latLng object for Leaflet to use
    odomLatLng: function () {
      return L.latLng(this.odom.latitude_deg + this.odom.latitude_min/60, this.odom.longitude_deg + this.odom.longitude_min/60)
    },
    
    // Concat waypoints on course with rover marker at index 0 for polyline
    polylinePath: function () {
      return [this.odomLatLng].concat(this.route.map(waypoint => waypoint.latLng))
    },
    
    // Concat waypoints on projected course with rover marker at index 0 for polyline
    projectedPath: function () {
      return [this.odomLatLng].concat(this.projectedPoints.map(projected_point => projected_point.latLng))
    },

  },

  data () {
    return {
      center: L.latLng(38.406371, -110.791954),
      attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
      online: true,
      onlineUrl: onlineUrl,
      offlineUrl: offlineUrl,
      roverMarker: null,
      waypointIcon: null,
      map: null,
      odomCount: 0,
      locationIcon: null,
      odomPath: [],

      tangentMarker: null,
      tangentIcon: null,

      targetBearingMarker: null,
      targetBearingIcon: null,

      projectedPoints: [],
      projectedPointsType: '',

      post1: null,
      post2: null,

      findRover: false,

      options: {
        type: Object,
        default: () => ({})
      },
      tileLayerOptions: {
        maxNativeZoom: 18,
        maxZoom: 100
      }
    }
  },

  props: {
    odom: {
      type: Object,
      required: true
    },
    GPS: {
      type: Object,
      required: true
    },
    TargetBearing: {
      type: Object,
      required: true
    }
  },

  methods: {
    // Event listener for setting store values to get data to waypoint Editor
    getClickedLatLon: function (e) {
      this.setClickPoint(
          { 
            lat: e.latlng.lat,
            lon: e.latlng.lng
          }
        )
    },

    ...mapMutations('autonomy',{
      setClickPoint: 'setClickPoint',
      setWaypointList: 'setWaypointList',
      setOdomFormat: 'setOdomFormat'
    }),
  },


  watch: {
    odom: {
      handler: function (val) {
        // Trigger every time rover odom is changed
        console.log("ODOM MESSAGE")
        console.log(val)
  
        const lat = val.latitude_deg + val.latitude_min / 60
        const lng = val.longitude_deg + val.longitude_min / 60
        const angle = val.bearing_deg
  
        const latLng = L.latLng(lat, lng)
  
        // Move to rover on first odom message
        if (!this.findRover) {
          this.findRover = true
          this.center = latLng
        }
        
        // Update the rover markers
        this.roverMarker.setRotationAngle(angle)
  
        this.roverMarker.setLatLng(latLng)
        // this.tangentMarker.setLatLng(latLng)
        // this.targetBearingMarker.setLatLng(latLng)
  
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

    // Rotate tangent icon based on bearing
    GPS: function (val) {
      this.tangentMarker.setRotationAngle(val.bearing_deg)
    },


    // Rotate target icon based on bearing
    TargetBearing: function (val) {
      this.targetBearingMarker.setRotationAngle(val.target_bearing)
    },

  },

  // Pull objects from refs to be able to access data and change w functions
  mounted: function () {
    this.$nextTick(() => {
      console.log(this.$refs)
      this.map = this.$refs.map.mapObject
      this.roverMarker = this.$refs.rover.mapObject
      // More Tangent Marker stuff
      // this.tangentMarker = this.$refs.tangent.mapObject
      // this.targetBearingMarker = this.$refs.target_bearing.mapObject
    })
  }
}
</script>

<style scoped>
  
  .controls label{
  font-size: 12px;
}

.controls div{
  display: inline-block;
}

.online{
  float:right;
}

.wrap {
  align-items: center;
  height: 100%;
  display: grid;
  overflow:hidden;
  min-height: 100%;
  grid-gap: 3px;
  grid-template-columns: 1fr;
  grid-template-rows: 94% 6%;
  grid-template-areas:"map" 
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
