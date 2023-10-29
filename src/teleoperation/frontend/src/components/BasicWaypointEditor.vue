<template>
    <div class="wrap">
        <div class="identification">Name: <input v-model="name" size="15" /></div>
        <br />
        <input v-model="odom_format_in" type="radio" value="D" class="checkbox" />
        <font size="2">D</font>
        <input
          v-model="odom_format_in"
          type="radio"
          value="DM"
          class="checkbox"
        />
        <font size="2">DM</font>
        <input
          v-model="odom_format_in"
          type="radio"
          value="DMS"
          class="checkbox"
        />
        <font size="2">DMS</font><br />
        <div class="wp-input">
          <p><input v-model.number="input.lat.d" size="13" />º</p>
          <p v-if="min_enabled">
            <input v-model.number="input.lat.m" size="13" />'
          </p>
          <p v-if="sec_enabled">
            <input v-model.number="input.lat.s" size="13" />"
          </p>
          N
        </div>
        <div class="wp-input">
          <p><input v-model.number="input.lon.d" size="13" />º</p>
          <p v-if="min_enabled">
            <input v-model.number="input.lon.m" size="13" />'
          </p>
          <p v-if="sec_enabled">
            <input v-model.number="input.lon.s" size="13" />"
          </p>
          E
        </div>
        <br />
        <div style="display: inline-block">
          <button @click="addWaypoint(input)">Add Waypoint</button>
          <button @click="addWaypoint(formatted_odom)">Drop Waypoint</button>
        </div>
        <div class="all-waypoints">
          <h4 class="waypoint-headers">Waypoints</h4>
          <button @click="clearWaypoint">Clear Waypoints</button>
        </div>
        <draggable v-model="storedWaypoints" class="dragArea" draggable=".item'">
          <WaypointItem
            v-for="(waypoint, i) in storedWaypoints"
            :key="i"
            :waypoint="waypoint"
            :index="i"
            @delete="deleteItem($event)"
            @find="findWaypoint($event)"
          />
        </draggable>
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
    position: relative;
    display: flex;
    flex-direction: row;
    height: 100%;
    margin: auto;
  }
  
  .dragArea {
    height: 100%;
  }
  
  .identification {
    display: inline-block;
  }
  
  .all-waypoints {
    display: inline-flex;
  }
  
  .all-waypoints button {
    margin: 5px;
    width: 115px;
    height: 20px;
  }
  
  .joystick {
    grid-area: joystick;
  }
  
  .wp-input p {
    display: inline;
  }
  
  .waypoint-headers {
    margin: 5px 0px 0px 5px;
  }
  </style>