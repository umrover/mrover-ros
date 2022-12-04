<template>
  <div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>SA Dashboard</h1>
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
    <div class="box map light-bg">  
      <SARoverMap v-bind:odom="odom"/>
    </div>
    <div class="box waypoints light-bg">
      <SAWaypointEditor v-bind:odom="odom"/>
    </div>
  </div>
</template>

<script>
import SARoverMap from './SARoverMap.vue';
import SAWaypointEditor from './SAWaypointEditor.vue'


  export default {
    data() {
      return {
        // Default coordinates are at NC 53 Parking Lot
        odom: {
          latitude_deg: 42.294864932393835,
          longitude_deg: -83.70781314674628,
          bearing_deg: 0
        }
      }
    },

    components:{
      SARoverMap,
      SAWaypointEditor
    }
  }
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
  .wrapper {
    display: grid;
    overflow: hidden;
    min-height: 98vh;
    grid-gap: 10px;
    grid-template-columns: 85vh auto auto;
    grid-template-rows: 60px auto;
    grid-template-areas: "header header header"
                         "map waypoint waypoint";
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

  .cameras {
    overflow: auto;
  }

  .map {
    grid-area: map;
  }

  .Joystick {
    font-size: 1em;
    height: 41%;
    width: 93%;
    display: inline-block;
  }
  .raw-sensors{
    font-size: 1em;
  }

  .controls {
    font-size: 1em;
    height: 40.5%;
    display: block;
  }
  ul#vitals li {
    display: inline;
    float: left;
    padding: 0px 10px 0px 0px;
  }
</style>