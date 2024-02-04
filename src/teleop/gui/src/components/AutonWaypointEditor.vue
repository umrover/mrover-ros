<template>
  <div class="wrap">
    <div class="col-wrap" style="left: 0">
      <div class="box">
        <div class="identification">
          Name: <input v-model="name" size="15" /> ID:
          <input v-model="id" type="number" max="249" min="-1" step="1" />
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
          <button @click="openModal">Competition Waypoint Entry</button>
        </div>
      </div>
      <div class="box1">
        <div class="all-waypoints">
          <h4 class="waypoint-headers">All Waypoints</h4>
          <button @click="clearWaypoint">Clear Waypoints</button>
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
      <div class="box datagrid">
        <div class="auton-check">
          <AutonModeCheckbox
            ref="autonCheckbox"
            :name="autonButtonText"
            :color="autonButtonColor"
            @toggle="toggleAutonMode($event)"
          />
          <Checkbox
            ref="teleopCheckbox"
            class="teleop-checkbox"
            :name="'Teleop Controls'"
            @toggle="toggleTeleopMode($event)"
          />
        </div>
        <div class="stuck-check">
          <Checkbox
            class="stuck-checkbox"
            name="Stuck"
            @toggle="roverStuck = !roverStuck"
          ></Checkbox>
        </div>
        <div class="stats">
          <VelocityCommand />
        </div>
      </div>
      <div class="box1">
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

    <div v-if="showModal" @close="showModal = false">
      <transition name="modal-fade">
        <div class="modal-backdrop">
          <div class="modal" role="dialog">
            <header id="modalTitle" class="modal-header">
              <h4>Enter your waypoints:</h4>
              <button
                type="button"
                class="btn-close"
                @click="showModal = false"
              >
                x
              </button>
            </header>

            <section id="modalDescription" class="modal-body">
              <slot name="body">
                <div class="modal-odom-format">
                  <h5>Waypoint format:</h5>
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
                </div>
                <div
                  v-for="(header, index) in compModalHeaders"
                  :key="header"
                  class="comp-modal-inputs"
                >
                  <h5>{{ header }}</h5>
                  <input
                    id="lat_deg"
                    v-model.number="compModalLatDeg[index]"
                    type="number"
                    value="0"
                    min="-90"
                    max="90"
                  />
                  <label>º</label>
                  <input
                    v-if="min_enabled"
                    id="lat_min"
                    v-model.number="compModalLatMin[index]"
                    type="number"
                    value="0"
                    min="0"
                    max="60"
                  />
                  <label v-if="min_enabled">'</label>
                  <input
                    v-if="sec_enabled"
                    id="lat_sec"
                    v-model.number="compModalLatSec[index]"
                    type="number"
                    value="0"
                    min="0"
                    max="3600"
                  />
                  <label v-if="sec_enabled">"</label>
                  <label>N &nbsp; &nbsp;</label>
                  <input
                    id="lon_deg"
                    v-model.number="compModalLonDeg[index]"
                    type="number"
                    value="0"
                    min="-180"
                    max="180"
                  />
                  <label>º</label>
                  <input
                    v-if="min_enabled"
                    id="lon_min"
                    v-model.number="compModalLonMin[index]"
                    type="number"
                    value="0"
                    min="0"
                    max="60"
                  />
                  <label v-if="min_enabled">'</label>
                  <input
                    v-if="sec_enabled"
                    id="lon_sec"
                    v-model.number="compModalLonSec[index]"
                    type="number"
                    value="0"
                    min="0"
                    max="3600"
                  />
                  <label v-if="sec_enabled">"</label>
                  <label>E</label>
                </div>
              </slot>
            </section>

            <footer class="modal-footer">
              <button
                type="button"
                :disabled="!comp_modal_able_to_submit"
                @click="submitModal()"
              >
                Submit
              </button>
            </footer>
          </div>
        </div>
      </transition>
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

let stuck_interval, intermediate_publish_interval;

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

      showModal: false,
      compModalHeaders: [
        "Start",
        "Waypoint 1",
        "Waypoint 2",
        "Waypoint 3",
        "Post 1",
        "Post 2",
        "Post 3",
        "Gate",
      ],
      compModalLatDeg: Array(8).fill(0),
      compModalLatMin: Array(8).fill(0),
      compModalLatSec: Array(8).fill(0),
      compModalLonDeg: Array(8).fill(0),
      compModalLonMin: Array(8).fill(0),
      compModalLonSec: Array(8).fill(0),

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

    comp_modal_able_to_submit: function () {
      // Ensure that all inputs in modal are valid before submitting
      return (
        this.compModalLatDeg.every((val) => Number.isFinite(val)) &&
        this.compModalLatMin.every((val) => Number.isFinite(val)) &&
        this.compModalLatSec.every((val) => Number.isFinite(val)) &&
        this.compModalLonDeg.every((val) => Number.isFinite(val)) &&
        this.compModalLonMin.every((val) => Number.isFinite(val)) &&
        this.compModalLonSec.every((val) => Number.isFinite(val))
      );
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

  beforeUnmount: function () {
    window.clearInterval(stuck_interval);
    window.clearInterval(intermediate_publish_interval);
    this.autonEnabled = false;
    this.sendEnableAuton();
  },

  created: function () {
    this.course_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/intermediate_enable_auton",
      messageType: "mrover/EnableAuton",
    });

    this.nav_status_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/nav_state",
      messageType: "std_msgs/String",
    });

    this.rover_stuck_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/rover_stuck",
      messageType: "std_msgs/Bool",
    });

    // Make sure local odom format matches vuex odom format
    this.odom_format_in = this.odom_format;

    this.nav_status_sub.subscribe((msg) => {
      // If still waiting for nav...
      if ((msg.data == "OffState" && this.autonEnabled) ||
          (msg.data !== "OffState" && !this.autonEnabled)) {
        return;
      }

      this.waitingForNav = false;
      this.autonButtonColor = this.autonEnabled ? "green" : "red";
    });

    // Interval for publishing stuck status for training data
    stuck_interval = window.setInterval(() => {
      this.rover_stuck_pub.publish({ data: this.roverStuck });
    }, 100);

    intermediate_publish_interval = window.setInterval(() => {
      this.sendEnableAuton();
    }, 1000);
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
      // If Auton Enabled send course
      if (this.autonEnabled) {
        this.course_pub.publish({
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
          enable: true
        });
      } else {
        // Else send false and no array
        this.course_pub.publish({waypoints: [], enable: false});
      }
    },

    openModal: function () {
      this.showModal = true;
      // Reset compModal Arrays
      this.compModalLatDeg = Array(8).fill(0);
      this.compModalLatMin = Array(8).fill(0);
      this.compModalLatSec = Array(8).fill(0);
      this.compModalLonDeg = Array(8).fill(0);
      this.compModalLonMin = Array(8).fill(0);
      this.compModalLonSec = Array(8).fill(0);
    },

    submitModal: function () {
      this.showModal = false;
      // Create lat/lon objects from comp modal arrays
      const coordinates = this.compModalLatDeg.map((deg, i) => {
        return {
          lat: {
            d: deg,
            m: this.compModalLatMin[i],
            s: this.compModalLatSec[i],
          },
          lon: {
            d: this.compModalLonDeg[i],
            m: this.compModalLonMin[i],
            s: this.compModalLonSec[i],
          },
        };
      });

      let coord_num = 0;
      let tag_id = 0;

      // Start AR tag is always 0.
      this.storedWaypoints.push({
        name: "Start",
        id: tag_id,
        lat: convertDMS(coordinates[coord_num].lat, "D").d,
        lon: convertDMS(coordinates[coord_num].lon, "D").d,
        gate: false,
        post: false,
      });

      ++coord_num;
      ++tag_id;

      // Add Waypoints, which we set as sentinel value -1.
      for ( ; coord_num < 4; ++coord_num) {
        this.storedWaypoints.push({
          name: "Waypoint " + coord_num,
          id: -1,
          lat: convertDMS(coordinates[coord_num].lat, "D").d,
          lon: convertDMS(coordinates[coord_num].lon, "D").d,
          gate: false,
          post: false,
        });
      }

      // Add AR Tag Posts with IDs 1-3
      for ( ; coord_num < 7; ++coord_num) {
        this.storedWaypoints.push({
          name: "AR Tag Post " + tag_id,
          id: tag_id,
          lat: convertDMS(coordinates[coord_num].lat, "D").d,
          lon: convertDMS(coordinates[coord_num].lon, "D").d,
          gate: false,
          post: true,
        });

        ++tag_id;
      }

      // Add Gate Location with ID 4, meaning posts 4 and 5.
      this.storedWaypoints.push({
        name: "Gate",
        id: tag_id,
        lat: convertDMS(coordinates[coord_num].lat, "D").d,
        lon: convertDMS(coordinates[coord_num].lon, "D").d,
        gate: true,
        post: false,
      });
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
      // This will trigger the yellow "waiting for nav" state of the checkbox
      this.autonButtonColor = "yellow";
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

.col-wrap span {
  margin: -30px 0px 0px 0px;
}

.dragArea {
  height: 100%;
}

.identification {
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
  grid-gap: 5%;
  grid-template-columns: 1fr 1fr;
  grid-template-rows: 1fr 0.25fr;
  grid-template-areas:
  "auton-check stats"
  "teleop-check stuck-check";
  font-family: sans-serif;
  min-height: 16.3vh;
}

.all-waypoints {
  display: inline-flex;
}

.all-waypoints button {
  margin: 5px;
  width: 115px;
  height: 20px;
}

.wp-input p {
  display: inline;
}

.waypoint-headers {
  margin: 5px 0px 0px 5px;
}

/* Grid Area Definitions */
.auton-check {
  align-content: center;
  grid-area: auton-check;
}

.teleop-checkbox {
  grid-area: teleop-check;
  margin-top: -40px;
}

.stats {
  margin-top: 10px;
  grid-area: stats;
}

.stuck-check {
  align-content: center;
  grid-area: stuck-check;
  padding-inline: 20px;
}
.stuck-check .stuck-checkbox {
  transform: scale(1.5);
}

.odom {
  grid-area: odom;
}

/* Modal Classes */
/* TODO: Make Modal a Component or set up a package for modals */
.modal-backdrop {
  position: fixed;
  top: 0;
  bottom: 0;
  left: 0;
  right: 0;
  background-color: rgba(0, 0, 0, 0.3);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 100000;
}

.modal {
  background: #ffffff;
  box-shadow: 2px 2px 20px 1px;
  overflow-x: auto;
  display: flex;
  flex-direction: column;
  z-index: 10000;
}

.modal-header,
.modal-footer {
  padding: 5px;
  display: flex;
}

.modal-header {
  position: relative;
  border-bottom: 1px solid #eeeeee;
  justify-content: space-between;
}

.modal-footer {
  border-top: 1px solid #eeeeee;
  flex-direction: column;
}

.modal-body {
  display: flex;
  flex-direction: column;
  position: relative;
  padding: 20px 10px;
}

.btn-close {
  position: absolute;
  top: 0;
  right: 0;
  border: none;
  font-size: 20px;
  padding: 10px;
  cursor: pointer;
  font-weight: bold;
  color: #4aae9b;
  background: transparent;
}

.btn-green {
  color: white;
  background: #4aae9b;
  border: 1px solid #4aae9b;
  border-radius: 2px;
}

.modal-fade-enter,
.modal-fade-leave-to {
  opacity: 0;
}

.modal-fade-enter-active,
.modal-fade-leave-active {
  transition: opacity 0.5s ease;
}

.modal-odom-format {
  display: flex;
  flex-direction: row;
  justify-content: center;
  align-items: center;
}

.comp-modal-inputs input {
  width: 150px;
}
</style>
