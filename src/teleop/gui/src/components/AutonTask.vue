<template>
<div class="wrap">
    <div class="page_header">
        <h1>Auton Dashboard</h1>
        <img src="/static/new_mrover.png" alt="MRover" title="MRover" width="185" height="53" />
        <div class="help">
            <img src="/static/help.png" alt="Help" title="Help" width="48" height="48" />
        </div>
        <div class="helpscreen"></div>
        <div class="helpimages" style="display: flex; align-items: center; justify-content: space-evenly">
            <img src="/static/joystick.png" alt="Joystick" title="Joystick Controls" style="width: auto; height: 70%; display: inline-block" />
        </div>
    </div>
    <div class="box box1" :class="[nav_state_color]">
      <div>
        <h2>Nav State: {{ nav_status.nav_state_name }}</h2>
      </div>
      <div>
        <p style="margin-top: 6px">Joystick Values</p>
      </div>
      <div></div>
      <JoystickValues />
      <div class="box">
        <IMUCalibration />
      </div>
    </div>
    <div class="box map">
        <AutonRoverMap v-bind:odom="odom" />
    </div>
    <div class="box waypoints">
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
  </div>
</template>

<script>
import ROSLIB from "roslib";
import '../assets/style.css';
import AutonRoverMap from "./AutonRoverMap.vue";
import AutonWaypointEditor from "./AutonWaypointEditor.vue";
import DriveControls from "./DriveControls.vue";
import MastGimbalControls from "./MastGimbalControls.vue";
import { mapGetters } from "vuex";
import * as qte from "quaternion-to-euler";
import JoystickValues from "./JoystickValues.vue";
import IMUCalibration from "./IMUCalibration.vue";

const navBlue = "blue";
const navGreen = "green";
const navRed = "red";
const navGrey = "gray";

export default {
  components: {
    AutonRoverMap,
    AutonWaypointEditor,
    DriveControls,
    IMUCalibration,
    JoystickValues,
    MastGimbalControls,
  },

  data() {
    return {
      // Default coordinates are at NC 53 Parking Lot
      odom: {
        latitude_deg: 42.294864932393835,
        longitude_deg: -83.70781314674628,
        bearing_deg: 0,
      },

      nav_status: {
        nav_state_name: "OffState",
        completed_wps: 0,
        total_wps: 0,
      },

      enableAuton: {
        enable: false,
        GPSWaypoint: [],
      },

      teleopEnabledCheck: false,

      navBlink: false,
      greenHook: false,
      ledColor: "red",

      // Pubs and Subs
      nav_status_sub: null,
      odom_sub: null,
      auton_led_client: null,
      tfClient: null,
    };
  },

  computed: {
    ...mapGetters("autonomy", {
      autonEnabled: "autonEnabled",
      teleopEnabled: "teleopEnabled",
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
    },
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
    },
  },

  created: function () {
    this.nav_status_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/smach/container_status",
      messageType: "smach_msgs/SmachContainerStatus",
    });

    this.odom_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/gps/fix",
      messageType: "sensor_msgs/NavSatFix",
    });

    this.tfClient = new ROSLIB.TFClient({
      ros: this.$ros,
      fixedFrame: "map",
      // Thresholds to trigger subscription callback
      angularThres: 0.01,
      transThres: 0.01,
    });

    this.auton_led_client = new ROSLIB.Service({
      ros: this.$ros,
      name: "change_auton_led_state",
      serviceType: "mrover/ChangeAutonLEDState",
    });

    // Subscriber for odom to base_link transform
    this.tfClient.subscribe("base_link", (tf) => {
      // Callback for IMU quaternion that describes bearing
      let quaternion = tf.rotation;
      quaternion = [quaternion.w, quaternion.x, quaternion.y, quaternion.z];
      //Quaternion to euler angles
      let euler = qte(quaternion);
      // euler[2] == euler z component
      this.odom.bearing_deg = euler[2] * (180 / Math.PI);
    });

    this.nav_status_sub.subscribe((msg) => {
      // Callback for nav_status
      this.nav_status.nav_state_name = msg.active_states[0];
    });

    this.odom_sub.subscribe((msg) => {
      // Callback for latLng to be set
      this.odom.latitude_deg = msg.latitude;
      this.odom.longitude_deg = msg.longitude;
    });

    // Blink interval for green and off flasing
    setInterval(() => {
      this.navBlink = !this.navBlink;
    }, 500);

    // Initialize color to blue
    this.sendColor();
  },

  methods: {
    sendColor() {
      let request = new ROSLIB.ServiceRequest({
        color: this.ledColor,
      });

      this.auton_led_client.callService(request, (result) => {
        // Wait 1 second then try again if fail
        if (!result.success) {
          setTimeout(() => {
            this.sendColor();
          }, 1000);
        }
      });
    },
  },
};
</script>

<style scoped>
.wrap {
  display: grid;
  overflow:hidden;
  grid-gap: 10px;
  grid-template-columns: 2fr 1.25fr 0.75fr;
  grid-template-rows: auto 5fr 1fr;
  grid-template-areas:  "header header header"
                        "map waypoints waypoints"
                        "data waypoints waypoints";
  font-family: sans-serif;
  height: 100%;
  width: auto;
}

.box {
  box-shadow: 2px 2px 6px var(--shadow-color), -2px -2px 6px var(--shadow-color);
}
.box1 {
  display: grid;
  grid-template-columns: 40% 60%;
}

img {
  border: none;
  border-radius: 0px;
}

h2 {
  padding: 2px;
  margin: 0px;
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

.page_header {
    grid-area: header;
}
</style>
