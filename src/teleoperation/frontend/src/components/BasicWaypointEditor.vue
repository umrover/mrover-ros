<template>
    <div class="wrap">
      <div class="box">
        <div class="form-group">
          <label for="waypointname">Name:</label>
          <input class="form-control" id="waypointname" v-model="name">
        </div>

        <div class="form-check form-check-inline">
          <input v-model="odom_format_in" class="form-check-input" type="radio" id="radioD" value="D">
          <label class="form-check-label" for="radioD">D</label>
        </div>
        <div class="form-check form-check-inline">
          <input v-model="odom_format_in" class="form-check-input" type="radio" id="radioDM" value="DM">
          <label class="form-check-label" for="radioDM">DM</label>
        </div>
        <div class="form-check form-check-inline">
          <input v-model="odom_format_in" class="form-check-input" type="radio" id="radioDMS" value="DMS">
          <label class="form-check-label" for="radioDMS">DMS</label>
        </div>

        <div class="row">
          <div class="col input-group">
            <input class="form-control" id="deg1" v-model.number="input.lat.d">
            <span for="deg1" class="input-group-text">º</span>
          </div>
          <div v-if="min_enabled" class="col input-group">
            <input class="form-control" id="min1" v-model.number="input.lat.m">
            <span for="min1" class="input-group-text">'</span>
          </div>
          <div v-if="sec_enabled" class="col input-group">
            <input class="form-control" id="sec1" v-model.number="input.lat.s">
            <span for="sec1" class="input-group-text">"</span>
          </div>
          N
        </div>
        <div class="row">
          <div class="col input-group">
            <input class="form-control" id="deg2" v-model.number="input.lon.d">
            <span for="deg2" class="input-group-text">º</span>
          </div>
          <div v-if="min_enabled" class="col input-group">
            <input class="form-control" id="min2" v-model.number="input.lon.m">
            <span for="min2" class="input-group-text">'</span>
          </div>
          <div v-if="sec_enabled" class="col input-group">
            <input class="form-control" id="sec2" v-model.number="input.lon.s">
            <span for="sec2" class="input-group-text">"</span>
          </div>
          E
        </div>

        <div class="add-drop">
          <button class="btn btn-primary" @click="addWaypoint(input)">Add Waypoint</button>
          <button class="btn btn-primary" @click="addWaypoint(formatted_odom)">Drop Waypoint</button>
          <button class="btn btn-primary">Add Drone Position</button>
        </div>
      </div>
      <div class="box">
        <div class="all-waypoints">
          <h4 class="waypoint-headers">Waypoints</h4>
          <button class="btn btn-primary" @click="clearWaypoint">Clear Waypoints</button>
        </div>
        <!-- <draggable v-model="storedWaypoints" class="dragArea" draggable=".item'">
          <WaypointItem
            v-for="(waypoint, i) in storedWaypoints"
            :key="i"
            :waypoint="waypoint"
            :index="i"
            @delete="deleteItem($event)"
            @find="findWaypoint($event)"
          />
        </draggable> -->
      </div>
    </div>
  </template>
  
  <script lang="ts">
  import draggable from "vuedraggable";
  import { convertDMS } from "../utils.js";
  import WaypointItem from "./BasicWaypointItem.vue";
  import { mapMutations, mapGetters } from "vuex";
  import _ from "lodash";
  import L from "leaflet";
  
  export default {
    props: {
      odom: {
        type: Object,
        required: true
      }
    },
  
    data() {
      return {
        name: "Waypoint",
        odom_format_in: "DM",
        input: {
          lat: {
            d: 0,
            m: 0,
            s: 0
          },
          lon: {
            d: 0,
            m: 0,
            s: 0
          }
        },
  
        storedWaypoints: []
      };
    },
  
    methods: {
      ...mapMutations("erd", {
        setWaypointList: "setWaypointList",
        setHighlightedWaypoint: "setHighlightedWaypoint"
      }),
  
      ...mapMutations("map", {
        setOdomFormat: "setOdomFormat"
      }),
  
      deleteItem: function (payload: { index: any; }) {
        if (this.highlightedWaypoint == payload.index) {
          this.setHighlightedWaypoint(-1);
        }
        this.storedWaypoints.splice(payload.index, 1);
      },
  
      addWaypoint: function (coord: { lat: { d: number; m: number; s: number; }; lon: { d: number; m: number; s: number; }; }) {
        this.storedWaypoints.push({
          name: this.name,
          lat: (coord.lat.d + coord.lat.m / 60 + coord.lat.s / 3600).toFixed(5),
          lon: (coord.lon.d + coord.lon.m / 60 + coord.lon.s / 3600).toFixed(5)
        });
      },
  
      findWaypoint: function (payload: { index: any; }) {
        if (payload.index === this.highlightedWaypoint) {
          this.setHighlightedWaypoint(-1);
        } else {
          this.setHighlightedWaypoint(payload.index);
        }
      },
  
      clearWaypoint: function () {
        this.storedWaypoints = [];
      }
    },
  
    watch: {
      storedWaypoints: function (newList) {
        const waypoints = newList.map((waypoint: { lat: any; lon: any; name: any; }) => {
          return {
            latLng: L.latLng(waypoint.lat, waypoint.lon),
            name: waypoint.name
          };
        });
        this.setWaypointList(waypoints);
      },
  
      odom_format_in: function (newOdomFormat) {
        this.setOdomFormat(newOdomFormat);
        this.input.lat = convertDMS(this.input.lat, newOdomFormat);
        this.input.lon = convertDMS(this.input.lon, newOdomFormat);
      },
  
      clickPoint: function (newClickPoint) {
        this.input.lat.d = newClickPoint.lat;
        this.input.lon.d = newClickPoint.lon;
        this.input.lat.m = 0;
        this.input.lon.m = 0;
        this.input.lat.s = 0;
        this.input.lon.s = 0;
        this.input.lat = convertDMS(this.input.lat, this.odom_format_in);
        this.input.lon = convertDMS(this.input.lon, this.odom_format_in);
      }
    },
  
    created: function () {
      // Reset waypoint editors
      this.setHighlightedWaypoint(-1);
      this.setWaypointList([]);
  
      // Set odometer format
      this.odom_format_in = this.odom_format;
    },
  
    computed: {
      ...mapGetters("erd", {
        highlightedWaypoint: "highlightedWaypoint",
        clickPoint: "clickPoint"
      }),
  
      ...mapGetters("map", {
        odom_format: "odomFormat"
      }),
  
      min_enabled: function () {
        return this.odom_format != "D";
      },
  
      sec_enabled: function () {
        return this.odom_format == "DMS";
      },
  
      formatted_odom: function () {
        return {
          lat: convertDMS(
            { d: this.odom.latitude_deg, m: 0, s: 0 },
            this.odom_format
          ),
          lon: convertDMS(
            { d: this.odom.longitude_deg, m: 0, s: 0 },
            this.odom_format
          )
        };
      }
    },
  
    components: {
      draggable,
      WaypointItem
    }
  };
  </script>
  
  <style scoped>
  .wrap {
    display: flex;
    flex-direction: row;
    height: 100%;
    margin: auto;
  }

  .box {
    width: 50%;
    height: 100%;
    margin-right: 20px;
  }
  
  .dragArea {
    height: 100%;
  }
  
  .all-waypoints {
    display: inline-flex;
    align-items: center;
  }
  
  .all-waypoints button {
    margin: 5px;
  }
 
  .waypoint-headers {
    margin: auto;
  }

  .add-drop {
    display: flex;
    text-align: center;
  }

  .add-drop button{
    margin: 10px;
  }
  </style>