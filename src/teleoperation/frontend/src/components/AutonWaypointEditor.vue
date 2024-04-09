<template>
  <div class="wrap">
    <div class="col-wrap" style="left: 0">
      <div class="box">
        <div class="waypoint-header">
          <h4>All Waypoints</h4>
        </div>
        <div class="card w-75 col mb-3" v-for="waypoint in waypoints">
          <div class="card-body">
            <h5>{{ waypoint.name }}</h5>
            <p>ID: {{ waypoint.id }}  Type: {{ waypoint.type }}</p>
            <div class="row">
              <div class="col input-group">
                <input class="form-control" id="deg1" v-model.number="input.lat.d" />
                <span for="deg1" class="input-group-text">º</span>
              </div>
              <!-- <div v-if="min_enabled" class="col input-group">
                <input class="form-control" id="min1" v-model.number="input.lat.m" />
                <span for="min1" class="input-group-text">'</span>
              </div>
              <div v-if="sec_enabled" class="col input-group">
                <input class="form-control" id="sec1" v-model.number="input.lat.s" />
                <span for="sec1" class="input-group-text">"</span>
              </div> -->
              N
            </div>
            <div class="row">
              <div class="col input-group">
                <input class="form-control" id="deg2" v-model.number="input.lon.d" />
                <span for="deg2" class="input-group-text">º</span>
              </div>
              <!-- <div v-if="min_enabled" class="col input-group">
                <input class="form-control" id="min2" v-model.number="input.lon.m" />
                <span for="min2" class="input-group-text">'</span>
              </div>
              <div v-if="sec_enabled" class="col input-group">
                <input class="form-control" id="sec2" v-model.number="input.lon.s" />
                <span for="sec2" class="input-group-text">"</span>
              </div> -->
              E
            </div>
            <button class="btn btn-primary custom-btn" @click="addItem(waypoint)">Add Waypoint</button>
          </div>     
        </div>
        <!-- <div class="waypoints">
          <WaypointItem v-for="(waypoint, i) in storedWaypoints" :key="i" :waypoint="waypoint" :in_route="false"
            :index="i" @delete="deleteItem($event)"  @add="addItem($event)" />
        </div> -->
      </div>
    </div>
    <div class="col-wrap" style="left: 50%">
      <div class="datagrid">
        <AutonModeCheckbox ref="autonCheckbox" class="auton-checkbox" :name="autonButtonText" :color="autonButtonColor"
        @toggle="toggleAutonMode($event)" />
      <div class="stats">
        <VelocityCommand />
      </div>
      <Checkbox ref="teleopCheckbox" class="teleop-checkbox" :name="'Teleop Controls'"
        @toggle="toggleTeleopMode($event)" />
        </div>
      <h4 class="waypoint-headers my-3">Current Course</h4>
      <div class="box route">
        <WaypointItem v-for="(waypoint, i) in route" :id="id" :key="i" :waypoint="waypoint" :in_route="true" :index="i"
          :name="name" @delete="deleteItem($event)" />
      </div>
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

let stuck_interval: number, auton_publish_interval: number

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
      waypoints: [
        { 
          name: 'No Search',
          id: -1,
          type: 0 
        },
        {
          name: 'Post 1',
          id: 1,
          type: 1
        },
        {
          name: 'Post 2',
          id: 2,
          type: 1
        },
        {
          name: 'Post 3',
          id: 3,
          type: 1
        },
        {
          name: 'Mallet',
          id: -1,
          type: 2
        },
        {
          name: 'Water Bottle',
          id: -1,
          type: 3
        }],
      odom_format_in: 'D',
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

      teleopEnabledCheck: false,

      nav_status: {
        nav_state_name: 'Off',
        completed_wps: 0,
        total_wps: 0
      },

      route: [],

      autonButtonColor: 'btn-danger',

      roverStuck: false,
      waitingForNavResponse: false
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

    message: function (msg) {
      if (msg.type == 'nav_state') {
        // If still waiting for nav...
        if (
          (msg.state == 'OffState' && this.autonEnabled) ||
          (msg.state !== 'OffState' && !this.autonEnabled)
        ) {
          return
        }
        this.waitingForNavResponse = false
        this.autonButtonColor = this.autonEnabled ? 'btn-success' : 'btn-danger'
      } else if (msg.type == 'get_auton_waypoint_list') {
        // Get waypoints from server on page load
        this.waypoints = msg.data
        const waypoints = msg.data.map((waypoint: { lat: any; lon: any; name: any }) => {
          const lat = waypoint.lat
          const lon = waypoint.lon
          return { latLng: L.latLng(lat, lon), name: waypoint.name }
        })
        this.setWaypointList(waypoints)
      }
    },

    waypoints: {
      handler: function (newList) {
        const waypoints = newList.map((waypoint: { lat: any; lon: any; name: any }) => {
          const lat = waypoint.lat
          const lon = waypoint.lon
          return { latLng: L.latLng(lat, lon), name: waypoint.name }
        })
        this.setWaypointList(waypoints)
        this.sendMessage({ type: 'save_auton_waypoint_list', data: newList })
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
    window.clearInterval(auton_publish_interval)
    this.autonEnabled = false
    this.sendAutonCommand()
  },

  created: function () {
    // Make sure local odom format matches vuex odom format
    this.odom_format_in = this.odom_format

    auton_publish_interval = window.setInterval(() => {
      if (this.waitingForNavResponse) {
        this.sendAutonCommand()
      }
    }, 1000)
    window.setTimeout(() => {
      // Timeout so websocket will be initialized
      this.sendMessage({ type: 'get_auton_waypoint_list', data: null })
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
            (waypoint: { lat: number; lon: number; id: string; type: string }) => {
              const lat = waypoint.lat
              const lon = waypoint.lon
              // Return a GPSWaypoint.msg formatted object for each
              return {
                latitude_degrees: lat,
                longitude_degrees: lon,
                tag_id: parseInt(waypoint.id),
                type: parseInt(waypoint.type)
              }
            }
          )
        })
      } else {
        //if auton's not enabled, send an empty message
        this.sendMessage({ type: 'auton_command', is_enabled: false, waypoints: [] })
      }
    },

    deleteItem: function (waypoint: { index: any; in_route: boolean }) {
      if (this.highlightedWaypoint == waypoint.index) {
        this.setHighlightedWaypoint(-1)
      }
      if (!waypoint.in_route) {
        this.waypoints.splice(waypoint.index, 1)
      } else if (waypoint.in_route) {
        this.route.splice(waypoint.index, 1)
      }
    },

    findWaypoint: function (waypoint: { index: any }) {
      if (waypoint.index === this.highlightedWaypoint) {
        this.setHighlightedWaypoint(-1)
      } else {
        this.setHighlightedWaypoint(waypoint.index)
      }
    },

    // Add item from all waypoints div to current waypoints div
    addItem: function (waypoint: { in_route: boolean; index: number }) {
      if (!waypoint.in_route) {
        this.route.push(this.waypoints[waypoint.index])
      }
    },

    addWaypoint: function (coord: { lat: any; lon: any }) {
      if (this.type == 1 && !this.checkWaypointIDUnique(this.id)) {
        alert('Waypoint ID must be unique')
        return
      }
      this.waypoints.push({
        name: this.name,
        id: this.type == 1 ? this.id : -1, // Check if type is post, if so, set id to -1
        lat: convertDMS(coord.lat, 'D').d,
        lon: convertDMS(coord.lon, 'D').d,
        type: this.type,
        post: false
      })
    },

    checkWaypointIDUnique: function (id: any) {
      return this.waypoints.every((waypoint: { id: any }) => waypoint.id != id)
    },

    clearWaypoint: function () {
      this.waypoints = []
    },

    toggleAutonMode: function (val: boolean) {
      this.setAutonMode(val)
      // This will trigger the yellow "waiting for nav" state of the checkbox
      this.autonButtonColor = 'btn-warning'
      this.waitingForNavResponse = true
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
  width: 100%;
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
    'teleop-check stats';
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
  height: 43vh;
  overflow-y: auto;
}

.route {
  height: 60vh;
  overflow-y: scroll;
}

.wp-input p {
  display: inline;
}

/* Grid Area Definitions */
.teleop-checkbox {
  grid-area: teleop-check;
  width: 100%;
}

.stats {
  grid-area: stats;
}

.auton-checkbox {
  grid-area: auton-check;
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
</style>
