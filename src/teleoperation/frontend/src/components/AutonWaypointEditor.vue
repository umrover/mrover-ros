<template>
  <div class="wrap">
    <div class="col-wrap" style="left: 0">
      <div class="box">
        <div class="row">
          <div class="form-group col-md-6">
            <label for="waypointname">Name:</label>
            <input class="form-control" id="waypointname" v-model="name" />
          </div>
          <div class="form-group col-md-6">
            <label for="waypointid">ID:</label>
            <input
              v-if="type != 1"
              class="form-control"
              id="waypointid"
              v-model="id"
              type="number"
              max="249"
              min="0"
              step="1"
            />
            <input
              v-else
              class="form-control"
              id="waypointid"
              type="number"
              placeholder="-1"
              step="1"
              disabled
            />
          </div>
        </div>

        <select class="form-select my-3" v-model="type">
          <option value="0" selected>No Search</option>
          <option value="1" >Post</option>
          <option value="2">Mallet</option>
          <option value="3">Bottle</option>
        </select>

        <div class="form-check form-check-inline">
          <input
            v-model="odom_format_in"
            class="form-check-input"
            type="radio"
            id="radioD"
            value="D"
          />
          <label class="form-check-label" for="radioD">D</label>
        </div>
        <div class="form-check form-check-inline">
          <input
            v-model="odom_format_in"
            class="form-check-input"
            type="radio"
            id="radioDM"
            value="DM"
          />
          <label class="form-check-label" for="radioDM">DM</label>
        </div>
        <div class="form-check form-check-inline">
          <input
            v-model="odom_format_in"
            class="form-check-input"
            type="radio"
            id="radioDMS"
            value="DMS"
          />
          <label class="form-check-label" for="radioDMS">DMS</label>
        </div>

        <div class="row">
          <div class="col input-group">
            <input class="form-control" id="deg1" v-model.number="input.lat.d" />
            <span for="deg1" class="input-group-text">º</span>
          </div>
          <div v-if="min_enabled" class="col input-group">
            <input class="form-control" id="min1" v-model.number="input.lat.m" />
            <span for="min1" class="input-group-text">'</span>
          </div>
          <div v-if="sec_enabled" class="col input-group">
            <input class="form-control" id="sec1" v-model.number="input.lat.s" />
            <span for="sec1" class="input-group-text">"</span>
          </div>
          <div class="col">
              N
          </div>
        </div>
        <div class="row">
          <div class="col input-group">
            <input class="form-control" id="deg2" v-model.number="input.lon.d" />
            <span for="deg2" class="input-group-text">º</span>
          </div>
          <div v-if="min_enabled" class="col input-group">
            <input class="form-control" id="min2" v-model.number="input.lon.m" />
            <span for="min2" class="input-group-text">'</span>
          </div>
          <div v-if="sec_enabled" class="col input-group">
            <input class="form-control" id="sec2" v-model.number="input.lon.s" />
            <span for="sec2" class="input-group-text">"</span>
          </div>
          <div class="col">
              E
          </div>
        </div>

        <div class="add-drop">
          <button class="btn btn-primary" @click="addWaypoint(input)">Add Waypoint</button>
          <button class="btn btn-primary" @click="addWaypoint(formatted_odom)">
            Drop Waypoint
          </button>
          <!-- Disabled until Competion entry modal is redone -->
          <!-- <button class="btn btn-primary" @click="openModal">Competition Waypoint Entry</button> -->
        </div>
      </div>
      <div class="box">
        <div class="waypoint-header">
          <h4>All Waypoints</h4>
          <button class="btn btn-primary" @click="clearWaypoint">Clear Waypoints</button>
        </div>
        <div class="waypoints">
          <WaypointItem
            v-for="(waypoint, i) in storedWaypoints"
            :key="i"
            :waypoint="waypoint"
            :in_route="false"
            :index="i"
            @delete="deleteItem($event)"
            @add="addItem($event)"
          />
        </div>
      </div>
    </div>
    <div class="col-wrap" style="left: 50%">
      <AutonModeCheckbox
        ref="autonCheckbox"
        class="auton-checkbox"
        :name="autonButtonText"
        :color="autonButtonColor"
        @toggle="toggleAutonMode($event)"
      />
      <div class="stats">
        <VelocityCommand />
      </div>
      <Checkbox
        ref="teleopCheckbox"
        class="teleop-checkbox"
        :name="'Teleop Controls'"
        @toggle="toggleTeleopMode($event)"
      />
      <Checkbox class="stuck-checkbox" name="Stuck" @toggle="roverStuck = !roverStuck"></Checkbox>
      <div class="box">
        <h4 class="waypoint-headers">Current Course</h4>
        <WaypointItem
          v-for="(waypoint, i) in route"
          :id="id"
          :key="i"
          :waypoint="waypoint"
          :in_route="true"
          :index="i"
          :name="name"
          @delete="deleteItem($event)"
          @add="addItem($event)"
        />
      </div>
    </div>

    <div v-if="showModal" @close="showModal = false">
      <transition name="modal-fade">
        <div class="modal-backdrop">
          <div class="modal" role="dialog">
            <header id="modalTitle" class="modal-header">
              <h4>Enter your waypoints:</h4>
              <button type="button" class="btn-close" @click="showModal = false">x</button>
            </header>

            <section id="modalDescription" class="modal-body">
              <slot name="body">
                <div class="modal-odom-format">
                  <h5>Waypoint format:</h5>
                  <div class="form-check form-check-inline">
                    <input
                      v-model="odom_format_in"
                      class="form-check-input"
                      type="radio"
                      id="radioD"
                      value="D"
                    />
                    <label class="form-check-label" for="radioD">D</label>
                  </div>
                  <div class="form-check form-check-inline">
                    <input
                      v-model="odom_format_in"
                      class="form-check-input"
                      type="radio"
                      id="radioDM"
                      value="DM"
                    />
                    <label class="form-check-label" for="radioDM">DM</label>
                  </div>
                  <div class="form-check form-check-inline">
                    <input
                      v-model="odom_format_in"
                      class="form-check-input"
                      type="radio"
                      id="radioDMS"
                      value="DMS"
                    />
                    <label class="form-check-label" for="radioDMS">DMS</label>
                  </div>
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
                    min="-90"
                    max="90"
                  />
                  <label>º</label>
                  <input
                    v-if="min_enabled"
                    id="lat_min"
                    v-model.number="compModalLatMin[index]"
                    type="number"
                    min="0"
                    max="60"
                  />
                  <label v-if="min_enabled">'</label>
                  <input
                    v-if="sec_enabled"
                    id="lat_sec"
                    v-model.number="compModalLatSec[index]"
                    type="number"
                    min="0"
                    max="3600"
                  />
                  <label v-if="sec_enabled">"</label>
                  <label>N &nbsp; &nbsp;</label>
                  <input
                    id="lon_deg"
                    v-model.number="compModalLonDeg[index]"
                    type="number"
                    min="-180"
                    max="180"
                  />
                  <label>º</label>
                  <input
                    v-if="min_enabled"
                    id="lon_min"
                    v-model.number="compModalLonMin[index]"
                    type="number"
                    min="0"
                    max="60"
                  />
                  <label v-if="min_enabled">'</label>
                  <input
                    v-if="sec_enabled"
                    id="lon_sec"
                    v-model.number="compModalLonSec[index]"
                    type="number"
                    min="0"
                    max="3600"
                  />
                  <label v-if="sec_enabled">"</label>
                  <label>E</label>
                </div>
              </slot>
            </section>

            <footer class="modal-footer">
              <button type="button" :disabled="!comp_modal_able_to_submit" @click="submitModal()">
                Submit
              </button>
            </footer>
          </div>
        </div>
      </transition>
    </div>
  </div>
</template>

<script lang="ts">
import AutonModeCheckbox from './AutonModeCheckbox.vue'
import Checkbox from './Checkbox.vue'
import { convertDMS } from '../utils.js'
import VelocityCommand from './VelocityCommand.vue'
import WaypointItem from './AutonWaypointItem.vue'
import { mapState, mapActions, mapMutations, mapGetters } from 'vuex'
import _ from 'lodash'
import L from 'leaflet'

let stuck_interval: number, intermediate_publish_interval: number

export default {
  components: {
    WaypointItem,
    AutonModeCheckbox,
    Checkbox,
    VelocityCommand
  },

  props: {
    odom: {
      type: Object,
      required: true
    }
  },

  data() {
    return {
      name: 'Waypoint',
      id: '0',
      type: 0,
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

      showModal: false,
      compModalHeaders: [
        'Start',
        'Waypoint 1',
        'Waypoint 2',
        'Post 1',
        'Post 2',
        'Post 3',
      ],
      compModalLatDeg: Array(8).fill(0),
      compModalLatMin: Array(8).fill(0),
      compModalLatSec: Array(8).fill(0),
      compModalLonDeg: Array(8).fill(0),
      compModalLonMin: Array(8).fill(0),
      compModalLonSec: Array(8).fill(0),

      teleopEnabledCheck: false,

      nav_status: {
        nav_state_name: 'Off',
        completed_wps: 0,
        total_wps: 0
      },

      storedWaypoints: [],
      route: [],

      autonButtonColor: 'btn-danger',

      roverStuck: false
    }
  },
  computed: {
    ...mapState('websocket', ['message']),
    ...mapGetters('autonomy', {
      autonEnabled: 'autonEnabled',
      teleopEnabled: 'teleopEnabled',
      clickPoint: 'clickPoint'
    }),

    ...mapGetters('map', {
      odom_format: 'odomFormat'
    }),

    comp_modal_able_to_submit: function () {
      // Ensure that all inputs in modal are valid before submitting
      return (
        this.compModalLatDeg.every((val: unknown) => Number.isFinite(val)) &&
        this.compModalLatMin.every((val: unknown) => Number.isFinite(val)) &&
        this.compModalLatSec.every((val: unknown) => Number.isFinite(val)) &&
        this.compModalLonDeg.every((val: unknown) => Number.isFinite(val)) &&
        this.compModalLonMin.every((val: unknown) => Number.isFinite(val)) &&
        this.compModalLonSec.every((val: unknown) => Number.isFinite(val))
      )
    },

    formatted_odom: function () {
      return {
        lat: convertDMS({ d: this.odom.latitude_deg, m: 0, s: 0 }, this.odom_format),
        lon: convertDMS({ d: this.odom.longitude_deg, m: 0, s: 0 }, this.odom_format)
      }
    },

    min_enabled: function () {
      return this.odom_format != 'D'
    },

    sec_enabled: function () {
      return this.odom_format == 'DMS'
    },

    autonButtonText: function () {
      return this.autonButtonColor == 'btn-warning'
        ? 'Setting to ' + this.autonEnabled
        : 'Autonomy Mode'
    }
  },

  watch: {
    message(msg) {
      if (msg.type == 'nav_state') {
        // If still waiting for nav...
        if (
          !(msg.state == 'OffState' && this.autonEnabled) &&
          !(msg.state !== 'OffState' && !this.autonEnabled)
        ) {
          this.autonButtonColor = this.autonEnabled ? 'btn-success' : 'btn-danger'
        }
      }
      else if (msg.type == 'get_waypoint_list') {
        // Get waypoints from server on page load
        this.storedWaypoints = msg.data
        const waypoints = msg.data.map((waypoint: { lat: any; lon: any; name: any }) => {
          const lat = waypoint.lat
          const lon = waypoint.lon
          return { latLng: L.latLng(lat, lon), name: waypoint.name }
        })
        this.setWaypointList(waypoints)
      }
    },

    route: {
      handler: function (newRoute) {
      const waypoints = newRoute.map((waypoint: { lat: any; lon: any; name: any }) => {
        const lat = waypoint.lat
        const lon = waypoint.lon
        return { latLng: L.latLng(lat, lon), name: waypoint.name }
      })
      this.setRoute(waypoints)
    },
      deep: true
    },

    storedWaypoints: { 
      handler: function (newList) {
      const waypoints = newList.map((waypoint: { lat: any; lon: any; name: any }) => {
        const lat = waypoint.lat
        const lon = waypoint.lon
        return { latLng: L.latLng(lat, lon), name: waypoint.name }
      })
      this.setWaypointList(waypoints)
      this.sendMessage({ type: 'save_waypoint_list', data: newList })
    },
      deep: true
    },

    odom_format_in: function (newOdomFormat) {
      this.setOdomFormat(newOdomFormat)
      this.input.lat = convertDMS(this.input.lat, newOdomFormat)
      this.input.lon = convertDMS(this.input.lon, newOdomFormat)
    },

    clickPoint: function (newClickPoint) {
      this.input.lat.d = newClickPoint.lat
      this.input.lon.d = newClickPoint.lon
      this.input.lat.m = 0
      this.input.lon.m = 0
      this.input.lat.s = 0
      this.input.lon.s = 0
      this.input.lat = convertDMS(this.input.lat, this.odom_format_in)
      this.input.lon = convertDMS(this.input.lon, this.odom_format_in)
    }
  },

  beforeUnmount: function () {
    window.clearInterval(stuck_interval)
    window.clearInterval(intermediate_publish_interval)
    this.autonEnabled = false
  },

  created: function () {
    // Make sure local odom format matches vuex odom format
    this.odom_format_in = this.odom_format

    window.setTimeout(() => {
      // Timeout so websocket will be initialized
      this.sendMessage({ type: 'get_waypoint_list', data: null })
    }, 250)
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    ...mapMutations('autonomy', {
      setRoute: 'setRoute',
      setWaypointList: 'setWaypointList',
      setHighlightedWaypoint: 'setHighlightedWaypoint',
      setAutonMode: 'setAutonMode',
      setTeleopMode: 'setTeleopMode'
    }),

    ...mapMutations('map', {
      setOdomFormat: 'setOdomFormat'
    }),

    sendAutonCommand() {
      if (this.autonEnabled) {
        this.sendMessage({
          type: 'auton_command',
          is_enabled: true,
          waypoints: _.map(
            this.route,
            (waypoint: { lat: number; lon: number; id: string; type: number }) => {
              const lat = waypoint.lat
              const lon = waypoint.lon
              // Return a GPSWaypoint.msg formatted object for each
              return {
                latitude_degrees: lat,
                longitude_degrees: lon,
                tag_id: parseInt(waypoint.id),
                type: waypoint.type
              }
            }
          )
        })
      } else {
        //if auton's not enabled, send an empty message
        this.sendMessage({ type: 'auton_command', is_enabled: false, waypoints: [] })
      }
    },

    openModal: function () {
      this.showModal = true
      // Reset compModal Arrays
      this.compModalLatDeg = Array(8).fill(0)
      this.compModalLatMin = Array(8).fill(0)
      this.compModalLatSec = Array(8).fill(0)
      this.compModalLonDeg = Array(8).fill(0)
      this.compModalLonMin = Array(8).fill(0)
      this.compModalLonSec = Array(8).fill(0)
    },

    submitModal: function () {
      // this.showModal = false
      // // Create lat/lon objects from comp modal arrays
      // const coordinates = this.compModalLatDeg.map((deg: any, i: string | number) => {
      //   return {
      //     lat: {
      //       d: deg,
      //       m: this.compModalLatMin[i],
      //       s: this.compModalLatSec[i]
      //     },
      //     lon: {
      //       d: this.compModalLonDeg[i],
      //       m: this.compModalLonMin[i],
      //       s: this.compModalLonSec[i]
      //     }
      //   }
      // })

      // let coord_num = 0
      // let tag_id = 0

      // // Start AR tag is always 0.
      // this.storedWaypoints.push({
      //   type: 0,
      // })

      // ++coord_num
      // ++tag_id

      // // Add Waypoints, which we set as sentinel value -1.
      // for (; coord_num < 4; ++coord_num) {
      //   this.storedWaypoints.push({
      //     name: 'Waypoint ' + coord_num,
      //     id: -1,
      //     lat: convertDMS(coordinates[coord_num].lat, 'D').d,
      //     lon: convertDMS(coordinates[coord_num].lon, 'D').d,
      //     type: 0,
      //   })
      // }

      // // Add AR Tag Posts with IDs 1-3
      // for (; coord_num < 7; ++coord_num) {
      //   this.storedWaypoints.push({
      //     name: 'AR Tag Post ' + tag_id,
      //     id: tag_id,
      //     lat: convertDMS(coordinates[coord_num].lat, 'D').d,
      //     lon: convertDMS(coordinates[coord_num].lon, 'D').d,
      //     type: 1,
      //   })

      //   ++tag_id
      // }
    },

    deleteItem: function (waypoint: { index: any; in_route: boolean }) {
      if (this.highlightedWaypoint == waypoint.index) {
        this.setHighlightedWaypoint(-1)
      }
      if (!waypoint.in_route) {
        this.storedWaypoints.splice(waypoint.index, 1)
      } else if (waypoint.in_route) {
        this.route.splice(waypoint.index, 1)
      }
    },

    // Add item from all waypoints div to current waypoints div
    addItem: function (waypoint: { in_route: boolean; index: number }) {
      if (!waypoint.in_route) {
        this.route.push(this.storedWaypoints[waypoint.index])
      } else if (waypoint.in_route) {
        this.storedWaypoints.push(this.route[waypoint.index])
      }
    },

    addWaypoint: function (coord: { lat: any; lon: any }) {
      if (this.type != 1 && !this.checkWaypointIDUnique(this.id)) {
        alert('Waypoint ID must be unique')
        return
      }
      this.storedWaypoints.push({
        name: this.name,
        id: this.type != 1 ? this.id : -1, // Check if type is post, if so, set id to -1
        lat: convertDMS(coord.lat, 'D').d,
        lon: convertDMS(coord.lon, 'D').d,
        type: this.type,
        post: false
      })
    },

    checkWaypointIDUnique: function (id: any) {
      return this.storedWaypoints.every((waypoint: { id: any }) => waypoint.id != id)
    },

    clearWaypoint: function () {
      this.storedWaypoints = []
    },

    toggleAutonMode: function (val: any) {
      this.setAutonMode(val)
      // This will trigger the yellow "waiting for nav" state of the checkbox
      this.autonButtonColor = 'btn-warning'
      this.sendAutonCommand()
    },

    toggleTeleopMode: function () {
      this.teleopEnabledCheck = !this.teleopEnabledCheck
      this.sendMessage({ type: 'teleop_enabled', data: this.teleopEnabledCheck })
      this.$emit('toggleTeleop', this.teleopEnabledCheck)
    }
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
  height: 100%;
  width: 49.5%;
}

.dragArea {
  height: 100%;
}

.datagrid {
  display: grid;
  grid-gap: 5%;
  grid-template-columns: auto auto;
  grid-template-rows: auto auto;
  grid-template-areas:
    'auton-check stats'
    'teleop-check stuck-check';
  font-family: sans-serif;
}

.waypoint-header {
  display: inline-flex;
  align-items: center;
}

.waypoint-header button {
  margin: 5px;
}

.waypoint-header h4 {
  margin: 5px 0px 0px 5px;
}

.waypoints {
  height: 30%;
  overflow-y: hidden;
}

.wp-input p {
  display: inline;
}

/* Grid Area Definitions */
.auton-check {
  align-content: center;
  grid-area: auton-check;
}

.teleop-checkbox {
  grid-area: teleop-check;
  width: 50%;
  float: left;
  clear: both;
}

.stats {
  grid-area: stats;
  float: right;
  width: 50%;
}

.stuck-checkbox {
  grid-area: stuck-check;
  width: 50%;
  float: right;
}

.auton-checkbox {
  float: left;
  width: 50%;
}

.odom {
  grid-area: odom;
}

.add-drop {
  display: flex;
  text-align: center;
}

.add-drop button {
  margin: 10px;
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
