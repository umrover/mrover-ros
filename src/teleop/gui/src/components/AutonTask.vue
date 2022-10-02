<template>
<div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>Auton Dashboard</h1>
      <div class="spacer"></div>
      <!-- TODO: Add back comms indicators for ROSBridge Server Connection and Rover Connection -->
      <!-- <div class="comms">
        <ul id="vitals">
          <li><CommIndicator v-bind:connected="connections.websocket" name="Web Socket" /></li>
          <li><CommIndicator v-bind:connected="connections.lcm" name="Rover Connection Status" /></li>
        </ul>
      </div> -->
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
      <div class="raw-data raw-sensors">
        <!-- <RawSensorData v-bind:GPS=“GPS” v-bind:IMU=“IMU”/>
        <Obstacle v-bind:Obstacle=“Obstacle”/>
        <TargetList v-bind:TargetList=“TargetList”/>
        <DriveControls/>
        <DriveVelDataH/>
        <SaveAutonData v-bind:odom=“odom” v-bind:IMU=“IMU” v-bind:GPS=“GPS” v-bind:TargetBearing=“TargetBearing” v-bind:nav_status=“nav_status” v-bind:AutonDriveControl=“AutonDriveControl” v-bind:TargetList=“TargetList”/>
        <PlaybackAutonData/> -->
      </div>
    </div>
    <div class="box map light-bg">  
      <AutonRoverMap v-bind:odom="odom" v-bind:GPS="GPS" v-bind:TargetBearing="TargetBearing"/>
    </div>
    <div class="box waypoints light-bg">
      <AutonWaypointEditor v-bind:odom="odom" v-bind:AutonDriveControl="AutonDriveControl"/>
    </div>
</div>
</template>

<script>

import ROSLIB from "roslib"

let interval;

import AutonRoverMap from "./AutonRoverMap.vue"
import AutonWaypointEditor from './AutonWaypointEditor.vue'
import { mapGetters } from 'vuex';

const navBlue = "#4695FF"
const navGreen = "yellowgreen"
const navRed = "lightcoral"
const navGrey = "lightgrey"

export default {
  data() {
    return {

      odom: {
        latitude_deg: 0,
        latitude_min: 0,
        longitude_deg: 0,
        longitude_min: 0,
        bearing_deg: 0,
        speed: 0
      },

      // Default coordinates are at URC location
      GPS: {
        latitude_deg: 38,
        latitude_min: 24.38226,
        longitude_deg: -110,
        longitude_min: -47.51724,
        bearing_deg: 0,
        speed: 0
      },

      TargetBearing: {
        target_bearing: 0
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
      navBlink: false,
      greenHook: false,
      nav_status_sub: null,

    }
  },

  created: function() {
    this.nav_status_sub = new ROSLIB.Topic({
      ros : this.$ros,
      name : '/smach/container_status',
      messageType : 'smach_msgs/SmachContainerStatus'
    });

    this.nav_status_sub.subscribe((msg) => {
      this.nav_status.nav_state_name = msg.active_states[0]
      console.log('subscription running')
    });

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
      if(this.teleopEnabled){
        return navBlue
      }
      else if(true){ // add back this.autonEnabled
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
    },
  },

  watch: {
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
        // this.publish('/auton_led',ledMsg)
      }
    }
  },

  components:{
    AutonRoverMap,
    AutonWaypointEditor
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
  grid-template-rows: 50px 2fr 1fr 6vh 24vh;
  grid-template-areas: "header header header" 
                       "map waypoints waypoints"
                       "map waypoints waypoints" 
                       "data waypoints waypoints" 
                       "data angles odom";
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

