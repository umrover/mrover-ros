<template>
  <div class="wrap">
    <div class="col-wrap" style="left: 0">
      <div class="add-drop">
        <div class="identification">
          Name: <input v-model="name" class="box" size="15" /> ID:
          <input
            v-model="id"
            class="box"
            type="number"
            max="249"
            min="-1"
            step="1"
          />
        </div>
        <br />
        <input
          v-model="odom_format_in"
          type="radio"
          value="D"
          class="checkbox"
        /><font size="2">D</font>
        <input
          v-model="odom_format_in"
          type="radio"
          value="DM"
          class="checkbox"
        /><font size="2">DM</font>
        <input
          v-model="odom_format_in"
          type="radio"
          value="DMS"
          class="checkbox"
        /><font size="2">DMS</font><br />
        <div class="wp-input">
          <p><input v-model.number="input.lat.d" class="box" size="13" />º</p>
          <p v-if="min_enabled">
            <input v-model.number="input.lat.m" class="box" size="13" />'
          </p>
          <p v-if="sec_enabled">
            <input v-model.number="input.lat.s" class="box" size="13" />"
          </p>
          N
        </div>
        <div class="wp-input">
          <p><input v-model.number="input.lon.d" class="box" size="13" />º</p>
          <p v-if="min_enabled">
            <input v-model.number="input.lon.m" class="box" size="13" />'
          </p>
          <p v-if="sec_enabled">
            <input v-model.number="input.lon.s" class="box" size="13" />"
          </p>
          E
        </div>
        <div style="display: inline-block">
          <button class="button" @click="addWaypoint(input)">
            Add Waypoint
          </button>
          <button class="button" @click="addWaypoint(formatted_odom)">
            Drop Waypoint
          </button>
        </div>
      </div>
      <div class="box box1">
        <div class="all-waypoints">
          <h4 class="waypoint-headers">All Waypoints</h4>
          <button class="button" @click="clearWaypoint">Clear Waypoints</button>
        </div>
        <draggable
          v-model="storedWaypoints"
          class="dragArea"
          draggable=".item'"
        >
          <WaypointItem
            v-for="(waypoint, i) in storedWaypoints"
            :key="i"
            :waypoint="waypoint"
            :list="0"
            :index="i"
            @delete="deleteItem($event)"
            @toggleGate="toggleGate($event)"
            @togglePost="togglePost($event)"
            @add="addItem($event)"
            @find="findWaypoint($event)"
          />
        </draggable>
      </div>
    </div>
    <div class="col-wrap" style="left: 50%">
      <div class="datagrid">
        <div class="auton-check">
          <AutonModeCheckbox
            ref="autonCheckbox"
            :name="autonButtonText"
            :color="autonButtonColor"
            @toggle="toggleAutonMode($event)"
          />
          <Checkbox
            ref="teleopCheckbox"
            :name="'Teleop Controls'"
            @toggle="toggleTeleopMode($event)"
          />
        </div>
        <div class="stuck-check">
          <Checkbox name="Stuck" @toggle="roverStuck = !roverStuck"></Checkbox>
        </div>
        <div class="stats">
          <VelocityCommand />
        </div>
      </div>
      <div class="box box1">
        <h4 class="waypoint-headers">Current Course</h4>
        <draggable v-model="route" class="dragArea" draggable=".item'">
          <WaypointItem
            v-for="(waypoint, i) in route"
            :id="id"
            :key="i"
            :waypoint="waypoint"
            :list="1"
            :index="i"
            :name="name"
            @delete="deleteItem($event)"
            @toggleGate="toggleGate($event)"
            @togglePost="togglePost($event)"
            @add="addItem($event)"
            @find="findWaypoint($event)"
          />
        </draggable>
      </div>
    </div>
  </div>
</template>

<script>
import AutonModeCheckbox from "./AutonModeCheckbox.vue";
import Checkbox from "./Checkbox.vue";
import draggable from "vuedraggable";
import { convertDMS } from "../utils.js";
import VelocityCommand from "./VelocityCommand.vue";
import WaypointItem from "./AutonWaypointItem.vue";
import { mapMutations, mapGetters } from "vuex";
import _ from "lodash";
import L from "leaflet";
import ROSLIB from "roslib";
import "../assets/style.css";

let interval;

const WAYPOINT_TYPES = {
  NO_SEARCH: 0,
  POST: 1,
  GATE: 2,
};

export default {
  components: {
    draggable,
    WaypointItem,
    AutonModeCheckbox,
    Checkbox,
    VelocityCommand,
  },

  props: {
    odom: {
      type: Object,
      required: true,
    },
  },

  data() {
    return {
      name: "Waypoint",
      id: "-1",
      odom_format_in: "DM",
      input: {
        lat: {
          d: 0,
          m: 0,
          s: 0,
        },
        lon: {
          d: 0,
          m: 0,
          s: 0,
        },
      },

      teleopEnabledCheck: false,

      nav_status: {
        nav_state_name: "Off",
        completed_wps: 0,
        total_wps: 0,
      },

      storedWaypoints: [],
      route: [],

      autonButtonColor: "red",
      waitingForNav: false,

      roverStuck: false,

      //Pubs and Subs
      nav_status_sub: null,
      course_pub: null,

      rover_stuck_pub: null,
    };
  },
  computed: {
    ...mapGetters("autonomy", {
      autonEnabled: "autonEnabled",
      teleopEnabled: "teleopEnabled",
      clickPoint: "clickPoint",
    }),

    ...mapGetters("map", {
      odom_format: "odomFormat",
    }),

    formatted_odom: function () {
      return {
        lat: convertDMS(
          { d: this.odom.latitude_deg, m: 0, s: 0 },
          this.odom_format
        ),
        lon: convertDMS(
          { d: this.odom.longitude_deg, m: 0, s: 0 },
          this.odom_format
        ),
      };
    },

    min_enabled: function () {
      return this.odom_format != "D";
    },

    sec_enabled: function () {
      return this.odom_format == "DMS";
    },

    autonButtonText: function () {
      return this.autonButtonColor == "yellow"
        ? "Setting to " + this.autonEnabled
        : "Autonomy Mode";
    },
  },

  watch: {
    route: function (newRoute) {
      const waypoints = newRoute.map((waypoint) => {
        const lat = waypoint.lat;
        const lon = waypoint.lon;
        return { latLng: L.latLng(lat, lon), name: waypoint.name };
      });
      this.setRoute(waypoints);
    },

    storedWaypoints: function (newList) {
      const waypoints = newList.map((waypoint) => {
        const lat = waypoint.lat;
        const lon = waypoint.lon;
        return { latLng: L.latLng(lat, lon), name: waypoint.name };
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
    },
  },
  beforeDestroy: function () {
    window.clearInterval(interval);
  },

  created: function () {
    (this.course_pub = new ROSLIB.Service({
      ros: this.$ros,
      name: "/enable_auton",
      serviceType: "mrover/PublishEnableAuton",
    })),
      (this.nav_status_sub = new ROSLIB.Topic({
        ros: this.$ros,
        name: "/smach/container_status",
        messageType: "smach_msgs/SmachContainerStatus",
      })),
      (this.rover_stuck_pub = new ROSLIB.Topic({
        ros: this.$ros,
        name: "/rover_stuck",
        messageType: "std_msgs/Bool",
      })),
      this.nav_status_sub.subscribe(
        (msg) => {
          if (msg.active_states[0] !== "OffState" && !this.autonEnabled) {
            return;
          }
          this.waitingForNav = false;
          this.autonButtonColor = this.autonEnabled ? "green" : "red";
        },

        // Interval for publishing Course
        (interval = window.setInterval(() => {
          this.rover_stuck_pub.publish({ data: this.roverStuck });
        }, 100))
      );
  },

  mounted: function () {
    //Send auton off if GUI is refreshed
    this.sendEnableAuton();
  },

  methods: {
    ...mapMutations("autonomy", {
      setRoute: "setRoute",
      setWaypointList: "setWaypointList",
      setHighlightedWaypoint: "setHighlightedWaypoint",
      setAutonMode: "setAutonMode",
      setTeleopMode: "setTeleopMode",
    }),

    ...mapMutations("map", {
      setOdomFormat: "setOdomFormat",
    }),

    sendEnableAuton() {
      let course;

      // If Auton Enabled send course
      if (this.autonEnabled) {
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
              // WaypointType.msg format
              type: {
                val: waypoint.gate
                  ? WAYPOINT_TYPES.GATE
                  : waypoint.post
                  ? WAYPOINT_TYPES.POST
                  : WAYPOINT_TYPES.NO_SEARCH,
              },
              id: parseInt(waypoint.id),
            };
          }),
        };
      } else {
        // Else send false and no array
        course = {
          enable: false,
          waypoints: [],
        };
      }

      const course_request = new ROSLIB.ServiceRequest({
        enableMsg: course,
      });

      this.course_pub.callService(course_request, () => {});
    },

    deleteItem: function (payload) {
      if (this.highlightedWaypoint == payload.index) {
        this.setHighlightedWaypoint(-1);
      }
      if (payload.list === 0) {
        this.storedWaypoints.splice(payload.index, 1);
      } else if (payload.list === 1) {
        this.route.splice(payload.index, 1);
      }
    },

    findWaypoint: function (payload) {
      if (payload.index === this.highlightedWaypoint) {
        this.setHighlightedWaypoint(-1);
      } else {
        this.setHighlightedWaypoint(payload.index);
      }
    },

    // Add item from all waypoints div to current waypoints div
    addItem: function (payload) {
      if (payload.list === 0) {
        this.route.push(this.storedWaypoints[payload.index]);
      } else if (payload.list === 1) {
        this.storedWaypoints.push(this.route[payload.index]);
      }
    },

    toggleGate: function (payload) {
      if (payload.list === 0) {
        this.storedWaypoints[payload.index].gate =
          !this.storedWaypoints[payload.index].gate;
      } else if (payload.list === 1) {
        this.route[payload.index].gate = !this.route[payload.index].gate;
      }
    },

    togglePost: function (payload) {
      if (payload.list === 0) {
        this.storedWaypoints[payload.index].post =
          !this.storedWaypoints[payload.index].post;
      } else if (payload.list === 1) {
        this.route[payload.index].post = !this.route[payload.index].post;
      }
    },

    addWaypoint: function (coord) {
      this.storedWaypoints.push({
        name: this.name,
        id: this.id,
        lat: convertDMS(coord.lat, "D").d,
        lon: convertDMS(coord.lon, "D").d,
        gate: false,
        post: false,
      });
    },

    clearWaypoint: function () {
      this.storedWaypoints = [];
    },

    toggleAutonMode: function (val) {
      this.setAutonMode(val);
      // This will trigger the yellow "waiting for nav" state of the checkbox only if we are enabling the button
      this.autonButtonColor = val ? "yellow" : "red";
      this.waitingForNav = true;
      this.sendEnableAuton();
    },

    toggleTeleopMode: function () {
      this.teleopEnabledCheck = !this.teleopEnabledCheck;
      this.$emit("toggleTeleop", this.teleopEnabledCheck);
    },
  },
};
</script>

<style scoped>
.wrap {
  position: relative;
  display: flex;
  flex-direction: row;
  height: 100%;
}

.col-wrap {
  position: absolute;
  margin: 1.5px;
  display: inline-block;
  height: 100%;
  width: 49.5%;
}

.col-wrap span {
  margin: -30px 0px 0px 0px;
}

.dragArea {
  height: 100%;
}

.identification {
  display: inline-block;
}

.add-drop {
  height: 15%;
}

input {
  padding: 2px 0px 2px 5px;
  margin: 2px;
}

.box1 {
  padding: 0px 5px 0px 5px;
  overflow-y: scroll;
  height: 85%;
}

.button {
  width: auto;
  height: auto;
  margin: 5px;
}

.datagrid {
  display: grid;
  grid-gap: 5%;
  grid-template-columns: 1fr 1fr;
  grid-template-rows: 1fr 0.25fr;
  grid-template-areas:
    "auton-check stats"
    "auton-check stuck-check";
  font-family: sans-serif;
  min-height: min-content;
}

.all-waypoints {
  display: inline-flex;
}

.wp-input p {
  display: inline;
}

.waypoint-headers {
  margin: 5px 0px 0px 5px;
}

/* Grid Area Definitions */
.auton-check {
  display: block;
  text-align: center;
  grid-area: auton-check;
}

.stats {
  margin-top: 10px;
  grid-area: stats;
}

.stuck-check {
  text-align: center;
  grid-area: stuck-check;
}

.odom {
  grid-area: odom;
}
</style>
