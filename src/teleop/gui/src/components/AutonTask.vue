<template>
  <div>
    <div class="wrapper">
      <div class="box header">
        <img
          src="/static/mrover.png"
          alt="MRover"
          title="MRover"
          width="48"
          height="48"
        />
        <h1>Auton Dashboard</h1>
        <div class="spacer"></div>
        <MCUReset class="mcu_reset"></MCUReset>
        <div class="spacer"></div>
        <CommReadout class="comms"></CommReadout>
        <div class="help">
          <img
            src="/static/help.png"
            alt="Help"
            title="Help"
            width="48"
            height="48"
          />
        </div>
        <div class="helpscreen"></div>
        <div
          class="helpimages"
          style="
            display: flex;
            align-items: center;
            justify-content: space-evenly;
          "
        >
          <img
            src="/static/joystick.png"
            alt="Joystick"
            title="Joystick Controls"
            style="width: auto; height: 70%; display: inline-block"
          />
        </div>
      </div>
      <div class="box1 data" :style="{ backgroundColor: nav_state_color }">
        <div>
          <h2>Nav State: {{ nav_status.nav_state_name }}</h2>
        </div>
        <div>
          <p style="margin-top: 6px">Joystick Values</p>
        </div>
        <div></div>
        <JoystickValues />
        <div>
          <OdometryReading :odom="odom"></OdometryReading>
        </div>
      </div>
      <div class="box map light-bg">
        <AutonRoverMap :odom="odom" />
      </div>
      <div class="box waypoints light-bg">
        <AutonWaypointEditor
          :odom="odom"
          @toggleTeleop="teleopEnabledCheck = $event"
        />
      </div>
      <!--Enable the drive controls if auton is off-->
      <div
        v-if="!autonEnabled && teleopEnabledCheck"
        v-show="false"
        class="driveControls"
      >
        <DriveControls />
      </div>
      <div v-show="false">
        <MastGimbalControls></MastGimbalControls>
      </div>
      <div class="conditions">
        <div v-if="!stuck_status" class="stuck not-stuck">
          <h4>Nominal Conditions</h4>
        </div>
        <div v-else class="stuck rover-stuck">
          <h4>Obstruction Detected</h4>
        </div>
      </div>
      <div class="box1 cameras">
        <Cameras :primary="true" />
      </div>
      <div class="box1 moteus">
        <DriveMoteusStateTable :moteus-state-data="moteusState" />
        <JointStateTable :joint-state-data="jointState" :vertical="true" />
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import AutonRoverMap from "./AutonRoverMap.vue";
import AutonWaypointEditor from "./AutonWaypointEditor.vue";
import DriveControls from "./DriveControls.vue";
import MastGimbalControls from "./MastGimbalControls.vue";
import { mapGetters } from "vuex";
import JoystickValues from "./JoystickValues.vue";
import DriveMoteusStateTable from "./DriveMoteusStateTable.vue";
import JointStateTable from "./JointStateTable.vue";
import CommReadout from "./CommReadout.vue";
import Cameras from "./Cameras.vue";
import MCUReset from "./MCUReset.vue"

import { quaternionToMapAngle } from "../utils.js";
import OdometryReading from "./OdometryReading.vue";
const navBlue = "#4695FF";
const navGreen = "yellowgreen";
const navRed = "lightcoral";
const navGrey = "lightgrey";

const ledUpdateRate = 1;
let ledInterval;

export default {
  components: {
    AutonRoverMap,
    AutonWaypointEditor,
    DriveControls,
    JoystickValues,
    MastGimbalControls,
    CommReadout,
    Cameras,
    DriveMoteusStateTable,
    JointStateTable,
    MCUReset,
    OdometryReading
  },

  data() {
    return {
      // Default coordinates are at MDRS
      odom: {
        latitude_deg: 38.4060250,
        longitude_deg: -110.7923723,
        bearing_deg: 0
      },

      nav_status: {
        nav_state_name: "OffState",
        completed_wps: 0,
        total_wps: 0
      },

      enableAuton: {
        enable: false,
        GPSWaypoint: []
      },

      teleopEnabledCheck: false,

      navBlink: false,
      greenHook: false,
      ledColor: "red",

      stuck_status: false,

      brushless_motors_sub: null,

      // Default object isn't empty, so has to be initialized to ""
      moteusState: {
        name: ["", "", "", "", "", ""],
        error: ["", "", "", "", "", ""],
        state: ["", "", "", "", "", ""],
      },

      jointState: {},

      // Pubs and Subs
      nav_status_sub: null,
      odom_sub: null,
      stuck_sub: null,
      auton_led_pub: null,
      tfClient: null
    };
  },

  computed: {
    ...mapGetters("autonomy", {
      autonEnabled: "autonEnabled",
      teleopEnabled: "teleopEnabled"
    }),

    nav_state_color: function () {
      if (!this.autonEnabled && this.teleopEnabledCheck) {
        return navBlue;
      }
      if (this.nav_status.nav_state_name == "DoneState" && this.navBlink) {
        return navGreen;
      } else if (
        this.nav_status.nav_state_name == "DoneState" &&
        !this.navBlink
      ) {
        return navGrey;
      } else {
        return navRed;
      }
    }
  },

  watch: {
    // Publish auton LED color to ESW
    nav_state_color: function (color) {
      var send = true;
      if (color == navBlue) {
        this.ledColor = "blue";
      } else if (color == navRed) {
        this.ledColor = "red";
      } else if (color == navGreen || color == navGrey) {
        // Only send if previous color was not green
        send = !(this.ledColor == "green");
        this.ledColor = "green";
      }
      if (send) {
        this.sendColor();
      }
    }
  },

  beforeUnmount: function () {
    this.ledColor = "off";
    this.sendColor();
    window.clearInterval(ledInterval);
  },

  created: function () {
    this.nav_status_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/nav_state",
      messageType: "std_msgs/String"
    });

    this.odom_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/gps/fix",
      messageType: "sensor_msgs/NavSatFix"
    });

    this.stuck_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/stuck_status",
      messageType: "std_msgs/Bool"
    });

    this.tfClient = new ROSLIB.TFClient({
      ros: this.$ros,
      fixedFrame: "map",
      // Thresholds to trigger subscription callback
      angularThres: 0.0001,
      transThres: 0.01
    });

    this.auton_led_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "auton_led_cmd",
      messageType: "std_msgs/String"
    });

    // Subscriber for odom to base_link transform
    this.tfClient.subscribe("base_link", (tf) => {
      // Callback for IMU quaternion that describes bearing
      this.odom.bearing_deg = quaternionToMapAngle(tf.rotation);
    });

    this.nav_status_sub.subscribe((msg) => {
      // Callback for nav_status
      this.nav_status.nav_state_name = msg.data;
    });

    this.odom_sub.subscribe((msg) => {
      // Callback for latLng to be set
      this.odom.latitude_deg = msg.latitude;
      this.odom.longitude_deg = msg.longitude;
    });

    this.stuck_sub.subscribe((msg) => {
      // Callback for stuck status
      this.stuck_status = msg.data;
    });

    this.brushless_motors = new ROSLIB.Topic({
      ros: this.$ros,
      name: "drive_status",
      messageType: "mrover/MotorsStatus",
    });

    this.brushless_motors.subscribe((msg) => {
      this.jointState = msg.joint_states;
      this.moteusState = msg.moteus_states;
    });

    // Blink interval for green and off flasing
    setInterval(() => {
      this.navBlink = !this.navBlink;
    }, 500);

    // Initialize color to red.
    this.ledColor = "red";
    this.sendColor();

    ledInterval = window.setInterval(() => {
      this.sendColor();
    }, ledUpdateRate * 1000);
  },

  methods: {
    sendColor() {
      const msg = new ROSLIB.Message({
        data: this.ledColor
      });

      this.auton_led_pub.publish(msg);
    }
  }
};
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: 45vw 15vw auto;
  grid-template-rows: 60px 50vh auto auto auto;
  grid-template-areas:
    "header header header"
    "map map waypoints"
    "data data waypoints"
    "data data conditions"
    "cameras moteus moteus";

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
  height: 12 px;
  display: grid;
  grid-template-columns: 50% 50%;
}

.box2 {
  display: block;
}

.stuck {
  grid-area: stuck;
  border-radius: 5px;
  line-height: 40px;
  border: 1px solid black;
  font-size: 20px;
  text-align: center;
  justify-content: center;
}

.stuck h1 {
  margin-top: 30px;
}

.rover-stuck {
  background-color: lightcoral;
}

.not-stuck {
  background-color: yellowgreen;
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
  margin-right: 5px;
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
  opacity: 1;
  cursor: pointer;
}

.help:hover ~ .helpscreen,
.help:hover ~ .helpimages {
  visibility: visible;
}

/* Grid area declarations */
.map {
  grid-area: map;
}

.waypoints {
  grid-area: waypoints;
}

.conditions {
  grid-area: conditions;
}

.cameras {
  grid-area: cameras;
}

.moteus {
  grid-area: moteus;
}

.data {
  grid-area: data;
}

</style>
