<template>
<div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>Auton Dashboard</h1>
      <div class="spacer"></div>
      <div class="spacer"></div>
      <div class="help">
        <img src="/static/help.png" alt="Help" title="Help" width="48" height="48" />
      </div>
      <div class="helpscreen"></div>
      <div class="helpimages" style="display: flex; align-items: center; justify-content: space-evenly">
        <img src="/static/joystick.png" alt="Joystick" title="Joystick Controls" style="width: auto; height: 70%; display: inline-block" />
      </div>
    </div>
    <div class="box1 data" v-bind:style="{backgroundColor: nav_state_color}">
      <h2>Nav State: {{this.nav_status.nav_state_name}}</h2>
    </div>
    <div class="box map light-bg">  
      <AutonRoverMap v-bind:odom="odom"/>
    </div>
    <div class="box waypoints light-bg">
      <AutonWaypointEditor v-bind:odom="odom" v-bind:AutonDriveControl="AutonDriveControl"/>
    </div>
    <!--Enable the drive controls if auton is off-->
    <div class="driveControls" v-if="!this.autonEnabled" v-show="false">
      <DriveControls/>
    </div>
</div>
</template>

<script>
import ROSLIB from "roslib"
import AutonRoverMap from "./AutonRoverMap.vue"
import AutonWaypointEditor from './AutonWaypointEditor.vue'
import DriveControls from "./DriveControls.vue";
import { mapGetters } from 'vuex';
import * as qte from "quaternion-to-euler";

const navBlue = "#4695FF"
const navGreen = "yellowgreen"
const navRed = "lightcoral"
const navGrey = "lightgrey"

export default {
  data() {
    return {

      // Default coordinates are at NC 53 Parking Lot
      odom: {
        latitude_deg: 42.294864932393835,
        longitude_deg: -83.70781314674628,
        bearing_deg: 0
      },

      // Current Values being output to drivetrain for auton
      AutonDriveControl: {
        left_percent_velocity: 0,
        right_percent_velocity: 0
      },

      nav_status: {
        nav_state_name: "Off",
        completed_wps: 0,
        total_wps: 0
      },

      enableAuton: {
        enable: false,
        GPSWaypoint: []
      },

      navBlink: false,
      greenHook: false,

      // Pubs and Subs
      nav_status_sub: null,
      odom_sub: null,
      localization_sub: null

    }
  },

  created: function() {
    this.nav_status_sub = new ROSLIB.Topic({
      ros : this.$ros,
      name : '/smach/container_status',
      messageType : 'smach_msgs/SmachContainerStatus'
    });

    this.odom_sub = new ROSLIB.Topic({
      ros : this.$ros,
      name : '/gps/fix',
      messageType : 'sensor_msgs/NavSatFix'
    });

    this.localization_sub = new ROSLIB.Topic({
      ros : this.$ros,
      name : '/imu/data',
      messageType : 'sensor_msgs/Imu'
    });

    this.nav_status_sub.subscribe((msg) => {
      // Callback for nav_status
      this.nav_status.nav_state_name = msg.active_states[0]
    });

    this.odom_sub.subscribe((msg) => {
      // Callback for latLng to be set
      this.odom.latitude_deg = msg.latitude
      this.odom.longitude_deg = msg.longitude
    });

    this.localization_sub.subscribe((msg) => {
        // Callback for IMU quaternion that describes bearing
        let quaternion = msg.orientation
        quaternion = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
        //Quaternion to euler angles
        let euler = qte(quaternion)
        // euler[2] == euler z component
        this.odom.bearing_deg = euler[2] * (180/Math.PI)
    })

    setInterval(() => {
      this.navBlink = !this.navBlink
    }, 500)
  },

  computed: {
    ...mapGetters('autonomy', {
      autonEnabled: 'autonEnabled',
      teleopEnabled: 'teleopEnabled'
    }),
    
    nav_state_color: function() {
      if(!this.autonEnabled){
        return navBlue
      }
      else if(true){
        if(this.nav_status.nav_state_name == "Done" && this.navBlink){
          return navGreen
        }
        else if(this.nav_status.nav_state_name == "Done" && !this.navBlink){
          return navGrey
        }
        else{
          return navRed
        }
      }
      return navRed
    }
  },

  watch: {
    // Publish auton LED color to ESW
    nav_state_color: function(color){
      let ledMsg = {
        type: 'AutonLed',
        color: 'Null'
      }
      if(color == navBlue){
        ledMsg.color = 'Blue'
        this.greenHook = false
      }
      else if(color == navRed){
        ledMsg.color = 'Red'
        this.greenHook = false
      }
      else if(color == navGreen && !this.greenHook){
        ledMsg.color = 'Green'
        this.greenHook = true //Accounting for the blinking between navGrey and navGreen
      }
      if(!this.greenHook || ledMsg.color == 'Green'){
        // TODO: Implement this once ESW has this interface back up
        // this.publish('/auton_led',ledMsg)
      }
    },
  },

  components:{
    AutonRoverMap,
    AutonWaypointEditor,
    DriveControls
}
}
</script>

<style scoped>
.wrapper {
  display: grid;
  overflow:hidden;
  min-height: 98vh;
  grid-gap: 10px;
  grid-template-columns: 2fr 1.25fr 0.75fr;
  grid-template-rows: 50px 2fr 1fr 6vh;
  grid-template-areas: "header header header" 
                       "map waypoints waypoints"
                       "map waypoints waypoints" 
                       "data waypoints waypoints";
  font-family: sans-serif;
  height: auto;
  width: auto;
}

.box {
  border-radius: 5px;
  padding: 10px;
  border: 1px solid black;
}

.box1 {
  border-radius: 5px;
  background-color: LightGrey;
  padding: 10px;
  border: 1px solid black;
  overflow-y: scroll;
}

.box2 {
  display: block;
}

.light-bg {
  background-color: LightGrey;
}

img {
  border: none;
  border-radius: 0px;
}

.header {
  grid-area: header;
  display: flex;
  align-items: center;
}

.header h1 {
  margin-left: 5px;
}

h2 {
  padding: 2px;
  margin: 0px;
}

.spacer {
  flex-grow: 0.8;
}

.comms {
  display: flex;
  flex-direction: column;
  align-items: flex-start;
}

.comms * {
  margin-top: 2px;
  margin-bottom: 2px;
}

.helpscreen {
  z-index: 1000000000;
  display: block;
  visibility: hidden;
  background-color: black;
  opacity: 0.8;
  position: absolute;
  left: 0px;
  top: 0px;
  width: 100%;
  height: 100%;
}

.helpimages {
  z-index: 1000000001;
  visibility: hidden;
  position: absolute;
  left: 5%;
  top: 5%;
  width: 90%;
  height: 90%;
}

.help {
  z-index: 1000000002;
  display: flex;
  float: right;
  opacity: 0.8;
  cursor: auto;
}

.help:hover {
  opacity: 1.0;
  cursor: pointer;
}

.help:hover ~ .helpscreen, .help:hover ~ .helpimages {
  visibility: visible;
}

/* Grid area declarations */
.map {
  grid-area: map;
}

.waypoints {
  grid-area: waypoints;
}

</style>
