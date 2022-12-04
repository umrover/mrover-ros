<template>
  <div class="wrap">
    <div class="col-wrap" style="left: 0;">
      <div class="box">
        <div class="identification">
          Name: <input v-model="name" size="15">
          ID: <input v-model="id" type="number" max="249" min="-1" step="1">
        </div>
        <br>
        <input type="radio" v-model="odom_format_in" value="D" class="checkbox"><font size="2">D</font>
        <input type="radio" v-model="odom_format_in" value="DM" class="checkbox"><font size="2">DM</font>
        <input type="radio" v-model="odom_format_in" value="DMS" class="checkbox"><font size="2">DMS</font><br>
        <div class="wp-input">
          <p><input v-model.number="input.lat.d" size="13">º</p>
          <p v-if="this.min_enabled"><input v-model.number="input.lat.m" size="13">'</p>
          <p v-if="this.sec_enabled"><input v-model.number="input.lat.s" size="13">"</p>
          N
        </div>
        <div class="wp-input">
          <p><input v-model.number="input.lon.d" size="13">º</p>
          <p v-if="this.min_enabled"><input v-model.number="input.lon.m" size="13">'</p>
          <p  v-if="this.sec_enabled"><input v-model.number="input.lon.s" size="13">"</p>
          E
        </div>
        <br>
        <div style="display:inline-block">
          <button v-on:click="addWaypoint(input)">Add Waypoint</button>
          <button v-on:click="addWaypoint(formatted_odom)">Drop Waypoint</button>
        </div>
      </div>
      <div class="box1">
        <div class="all-waypoints">
          <h4 class="waypoint-headers">All Waypoints</h4>
          <button v-on:click="clearWaypoint">Clear Waypoints</button>
        </div>
        <draggable v-model="storedWaypoints" class="dragArea" draggable=".item'">
          <WaypointItem v-for="waypoint, i in storedWaypoints" :key="i" v-bind:waypoint="waypoint" 
            v-bind:list="0" v-bind:index="i" v-on:delete="deleteItem($event)"
            v-on:toggleGate="toggleGate($event)" v-on:togglePost="togglePost($event)" v-on:add="addItem($event)" v-on:find="findWaypoint($event)"/>
        </draggable>
      </div>
    </div>
    <div class="col-wrap" style="left: 50%">
      <div class="box datagrid">
        <div class="auton-check">
          <AutonModeCheckbox ref="autonCheckbox" v-bind:name="autonButtonText" v-bind:color="autonButtonColor"  v-on:toggle="toggleAutonMode($event)"/>
        </div>
        <div class="stuck-check">
          <Checkbox v-on:toggle="roverStuck=!roverStuck" name="Stuck"></Checkbox>
        </div>
        <div class="stats">
          <p>
            Waypoints Traveled: {{nav_status.completed_wps}}/{{nav_status.total_wps}}<br>
          </p>
        </div>
        <!-- TODO: Add back using ros topic data from /joystick -->
        <!-- <div class="joystick light-bg">
          <AutonJoystickReading v-bind:AutonDriveControl="AutonDriveControl"/>
        </div> -->
      </div>
      <div class="box1">
        <h4 class="waypoint-headers">Current Course</h4>
        <draggable v-model="route" class="dragArea" draggable=".item'">
          <WaypointItem v-for="waypoint, i in route" :key="i" v-bind:waypoint="waypoint" 
            v-bind:list="1" v-bind:index="i" v-bind:name="name" v-bind:id="id" v-on:delete="deleteItem($event)" 
            v-on:toggleGate="toggleGate($event)" v-on:togglePost="togglePost($event)" v-on:add="addItem($event)" v-on:find="findWaypoint($event)"/>
        </draggable>
      </div>
    </div>
  </div>
</template>

<script>
import AutonModeCheckbox from './AutonModeCheckbox.vue'
import Checkbox from './Checkbox.vue'
import draggable from 'vuedraggable'
import {convertDMS} from '../utils.js';
import WaypointItem from './AutonWaypointItem.vue'
import {mapMutations, mapGetters} from 'vuex'
import _ from 'lodash';
import fnvPlus from 'fnv-plus';
import L from 'leaflet'
import ROSLIB from 'roslib'

let interval;

export default {

  props: {
    odom: {
      type: Object,
      required: true
    },
    AutonDriveControl: {
      type: Object,
      required: true
    }
  },

  data () {
    return {
      name: "Waypoint",
      id: "-1",
      odom_format_in: 'DM',
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

      nav_status: {
        nav_state_name: "Off",
        completed_wps: 0,
        total_wps: 0
      },

      storedWaypoints: [],
      route: [],

      autonButtonColor: "red",
      waitingForNav: false,

      roverStuck: false,

      //Pubs and Subs
      nav_status_sub: null,
      course_pub: null,
      rover_stuck_pub: null
      
    }
  },

  beforeDestroy: function () {
    window.clearInterval(interval);
  },

  created: function () {

    this.course_pub = new ROSLIB.Topic({
          ros : this.$ros,
          name : '/auton/enable_state',
          messageType : 'mrover/EnableAuton'
    }),

    this.nav_status_sub = new ROSLIB.Topic({
          ros : this.$ros,
          name : '/smach/container_status',
          messageType : 'smach_msgs/SmachContainerStatus'
    }),

    this.rover_stuck_pub = new ROSLIB.Topic({
      ros : this.$ros,
      name : '/rover_stuck',
      messageType : 'std_msgs/Bool'
    }),
    
    this.nav_status_sub.subscribe((msg) => {
      if(msg.active_states[0] != "Off" && !this.autonEnabled){
          return
      }
      this.waitingForNav = false;
      this.autonButtonColor = this.autonEnabled ? "green" : "red";
    },

    // Interval for publishing Course
    interval = window.setInterval(() => {

      let course

      if(this.autonEnabled){ 
        course = {
          enable: true,
          // Map for every waypoint in the current route
          waypoints: _.map(this.route, (waypoint) => {
            const lat = waypoint.lat;
            const lon = waypoint.lon;

            // Return a GPSWaypoint.msg formatted object for each
            return {
              latitude_degrees: lat,
              longitude_degrees: lon,
              gate: waypoint.gate,
              post: waypoint.post,
              id: parseFloat(waypoint.id),
            }
          })
        };
      }
      else{ // Else send false and no array
        course = {
          enable: false,
          waypoints: []
        }
      }

      const courseMsg = new ROSLIB.Message(course)
      
      this.course_pub.publish(courseMsg)
      this.rover_stuck_pub.publish({data: this.roverStuck})
      
    }, 100));
  },

  methods: {
    ...mapMutations('autonomy',{
      setRoute: 'setRoute',
      setWaypointList: 'setWaypointList',
      setHighlightedWaypoint: 'setHighlightedWaypoint',
      setAutonMode: 'setAutonMode',
      setTeleopMode: 'setTeleopMode',
      setOdomFormat: 'setOdomFormat'
    }),

    deleteItem: function (payload) {
      if(this.highlightedWaypoint == payload.index){
        this.setHighlightedWaypoint(-1)
      }
      if(payload.list === 0) {
        this.storedWaypoints.splice(payload.index, 1)
      } else if(payload.list === 1) {
        this.route.splice(payload.index, 1)
      }
    },

    findWaypoint: function (payload) {
      if(payload.index === this.highlightedWaypoint){
        this.setHighlightedWaypoint(-1)
      }
      else{
        this.setHighlightedWaypoint(payload.index)
      }
    },

    // Add item from all waypoints div to current waypoints div
    addItem: function (payload) {
       if(payload.list === 0) {
        this.route.push(this.storedWaypoints[payload.index])
      } else if(payload.list === 1) {
        this.storedWaypoints.push(this.route[payload.index])
      }
    },

    toggleGate: function (payload) {
      if(payload.list === 0) {
        this.storedWaypoints[payload.index].gate = !this.storedWaypoints[payload.index].gate
      } else if(payload.list === 1) {
        this.route[payload.index].gate = !this.route[payload.index].gate
      }
    },

    togglePost: function (payload) {
      if(payload.list === 0) {
        this.storedWaypoints[payload.index].post = !this.storedWaypoints[payload.index].post
      } else if(payload.list === 1) {
        this.route[payload.index].post = !this.route[payload.index].post
      }
    },

    addWaypoint: function (coord) {
      this.storedWaypoints.push({
        name: this.name,
        id: this.id,
        lat: (convertDMS(coord.lat, "D").d),
        lon: (convertDMS(coord.lon, "D").d),
        gate: false,
        post: false,
      });
    },

    clearWaypoint: function () {
      this.storedWaypoints = [];
    },

    toggleAutonMode: function (val) {
      this.setAutonMode(val)
      this.autonButtonColor = "yellow"
      this.waitingForNav = true;
    },

  },

  watch: {
    route: function (newRoute) {
      const waypoints = newRoute.map((waypoint) => {
        const lat = waypoint.lat
        const lon = waypoint.lon
        return { latLng: L.latLng(lat, lon), name: waypoint.name }
      });
      this.setRoute(waypoints);
    },

    storedWaypoints: function (newList) {
      const waypoints = newList.map((waypoint) => {
        const lat = waypoint.lat
        const lon = waypoint.lon
        return { latLng: L.latLng(lat, lon), name: waypoint.name }
      });
      this.setWaypointList(waypoints);
    },

    odom_format_in: function (newOdomFormat) {
      this.setOdomFormat(newOdomFormat);
      this.input.lat = convertDMS(this.input.lat, newOdomFormat);
      this.input.lon = convertDMS(this.input.lon, newOdomFormat);
    },

    clickPoint: function (newClickPoint){
      this.input.lat.d = newClickPoint.lat
      this.input.lon.d = newClickPoint.lon
      this.input.lat.m = 0;
      this.input.lon.m = 0;
      this.input.lat.s = 0;
      this.input.lon.s = 0;
      this.input.lat = convertDMS(this.input.lat, this.odom_format_in);
      this.input.lon = convertDMS(this.input.lon, this.odom_format_in);
    }
  },

  computed: {
    ...mapGetters('autonomy', {
      autonEnabled: 'autonEnabled',
      teleopEnabled: 'teleopEnabled',
      odom_format: 'odomFormat',
      clickPoint: "clickPoint"
    }),

    formatted_odom: function() {
      return {
        lat: convertDMS({d: this.odom.latitude_deg, m: 0, s: 0}, this.odom_format),
        lon: convertDMS({d: this.odom.longitude_deg, m: 0, s: 0}, this.odom_format)
      }
    },

    min_enabled: function() {
      return this.odom_format != 'D';
    },

    sec_enabled: function() {
      return this.odom_format == 'DMS';
    },

    autonButtonText: function() {
      return (this.autonButtonColor == "yellow") ? "Setting to "+this.autonEnabled : "Autonomy Mode"
    }

  },

  components: {
    draggable,
    WaypointItem,
    AutonModeCheckbox,
    Checkbox,
  }

}
</script>

<style scoped>

  .wrap {
    position: relative;
    display: flex;
    flex-direction: row;
    height: 100%;
    margin: auto;
  }

  .col-wrap {
    position: absolute;
    margin: 1.5px;
    display: inline-block;
    /*flex-direction: column; */
    height: 100%;
    width: 49.5%;
  }
  
  .col-wrap span{
    margin:-30px 0px 0px 0px;
  }

  .dragArea {
    height: 100%;
  }

  .identification{
    display: inline-block;
  }

  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
    min-height: min-content;
    max-height: 32%;
    overflow: none;
    margin-bottom: 6px;
  }

  .box1 {
    border-radius: 5px;
    padding: 0px 5px 0px 5px;
    border: 1px solid black;
    overflow: scroll;
    min-height: min-content;
    max-height: 63%;
  }

  .datagrid {
      display: grid;
      grid-gap: 2px;
      grid-template-columns: 1fr 1fr;
      grid-template-rows: 1fr 0.25fr;
      grid-template-areas: "auton-check stats"
                           "teleop-check stuck-check";
      font-family: sans-serif;
      min-height: min-content;
  }

  .all-waypoints{
    display: inline-flex;
  }

  .all-waypoints button{
    margin: 5px;
    width: 115px;
    height: 20px;
  }

  .wp-input p {
    display: inline;
  }
  
  .waypoint-headers{
    margin: 5px 0px 0px 5px;
  }

  /* Grid Area Definitions */
  .auton-check{
    align-content: center;
    grid-area: auton-check;
  }
  
  .teleop-check{
    align-content: center;
    grid-area: teleop-check;
  }
  
  .stats{
    margin-top: -10px;
    grid-area: stats;
  }
  
  .stuck-check{
    align-content: center;
    grid-area: stuck-check;
  }
  
  .joystick{
    margin-top: -20px;
    grid-area: joystick;
  }
  
  .odom{
    grid-area: odom;
  }
  
</style>
