<template>
  <div class="wrap">
    <div class="col-wrap" style="left: 0">
      <div class="waypoint-header">
        <h4>All Waypoints</h4>
      </div>
      <button class="btn btn-primary" @click="openModal()">Add Waypoint From Map</button>
      <div class="waypoints">
        <div class="shadow p-3 my-2" v-for="(waypoint, index) in waypoints" :key="waypoint">
          <h5>{{ waypoint.name }}</h5>
          <p>ID: {{ waypoint.id }}</p>
          <div class="row">
            <div class="col input-group">
              <input class="form-control" id="deg1" v-model.number="waypoint.lat" />
              <span for="deg1" class="input-group-text">º</span>
            </div>
            N
          </div>
          <div class="row">
            <div class="col input-group">
              <input class="form-control" id="deg2" v-model.number="waypoint.lon" />
              <span for="deg2" class="input-group-text">º</span>
            </div>
            W
          </div>
          <button class="btn btn-primary" @click="addItem(waypoint)">Add Waypoint</button>
          <button v-if="index > 6" class="btn btn-primary mx-1" @click="deleteMapWaypoint(index)">Delete</button>
        </div>    
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
      <div class="route">
        <WaypointItem v-for="waypoint in route" :key="waypoint" :waypoint="waypoint" @delete="deleteItem(waypoint)" />
      </div>
    </div>
  </div>

  <div class="modal fade" id="modalWypt" tabindex="-1" role="dialog" aria-hidden="true">
        <div class="modal-dialog modal-dialog-centered" role="document">
          <div class="modal-content">
            <div class="modal-body">
              <div class="row">
                <div class="form-group col-md-6">
                  <label for="waypointname">Name:</label>
                  <input class="form-control" id="waypointname" v-model="modalWypt.name" />
                </div>
                <div class="form-group col-md-6">
                  <label for="waypointid">Tag ID:</label>
                  <input v-if="modalWypt.type == 1" class="form-control" id="waypointid" v-model="modalWypt.id" type="number" max="249" min="0"
                    step="1" />
                  <input v-else class="form-control" id="waypointid" type="number" placeholder="-1" step="1" disabled />
                </div>
                <select class="form-select my-3" v-model="modalWypt.type">
                  <option value="0" selected>No Search</option>
                  <option value="1">Post</option>
                  <option value="2">Mallet</option>
                  <option value="3">Water Bottle</option>
                </select>
              </div>
            </div>
            <div class="modal-footer">
              <button type="button" class="btn btn-secondary" @click="addMapWaypoint()">Add Waypoint</button>
            </div>
          </div>
        </div>
      </div>
</template>

<script lang="ts">
import AutonModeCheckbox from './AutonModeCheckbox.vue'
import Checkbox from './Checkbox.vue'
import VelocityCommand from './VelocityCommand.vue'
import WaypointItem from './AutonWaypointItem.vue'
import { mapState, mapActions, mapMutations, mapGetters } from 'vuex'
import _ from 'lodash'
import L from 'leaflet'
import { reactive } from 'vue'
import { Modal } from 'bootstrap'

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
          name: 'No Search 1',
          id: -1,
          type: 0, 
          lat: 0,
          lon: 0,
        },
        { 
          name: 'No Search 2',
          id: -1,
          type: 0, 
          lat: 0,
          lon: 0,
        },
        {
          name: 'Post 1',
          id: 1,
          type: 1,
          lat: 0,
          lon: 0,
        },
        {
          name: 'Post 2',
          id: 2,
          type: 1,
          lat: 0,
          lon: 0,
        },
        {
          name: 'Post 3',
          id: 3,
          type: 1,
          lat: 0,
          lon: 0,
        },
        {
          name: 'Mallet',
          id: -1,
          type: 2,
          lat: 0,
          lon: 0,
        },
        {
          name: 'Water Bottle',
          id: -1,
          type: 3,
          lat: 0,
          lon: 0,
        }],
      
      modal: null,
      modalWypt: {
          name: '',
          id: -1,
          type: 0,
          lat: 0,
          lon: 0,
        },

      teleopEnabledCheck: false,

      nav_status: {
        nav_state_name: 'Off',
        completed_wps: 0,
        total_wps: 0
      },

      route: reactive([]),

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

    autonButtonText: function () {
      return this.autonButtonColor == 'btn-warning'
        ? 'Setting to ' + this.autonEnabled
        : 'Autonomy Mode'
    }
  },

  watch: {
    waypoints: {
      handler: function (newList) {
        const waypoints = newList.map((waypoint) => {
          const lat = waypoint.lat
          const lon = waypoint.lon
          return { latLng: L.latLng(lat, lon), name: waypoint.name }
        })
        this.setWaypointList(waypoints)
        this.sendMessage({ type: 'save_auton_waypoint_list', data: newList })
      },
      deep: true
    },

    route: {
      handler: function (newRoute) {
          const waypoints = newRoute.map((waypoint) => {
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
        if(msg.data.length > 0) this.waypoints = msg.data 
        const waypoints = msg.data.map((waypoint: { lat: any; lon: any; name: any }) => {
          const lat = waypoint.lat
          const lon = waypoint.lon
          return { latLng: L.latLng(lat, lon), name: waypoint.name }
        })
        this.setWaypointList(waypoints)
      }
    },
  },

  mounted() {
    this.modal = new Modal('#modalWypt', {})
  },

  beforeUnmount: function () {
    window.clearInterval(stuck_interval)
    window.clearInterval(auton_publish_interval)
    this.autonEnabled = false
    this.sendAutonCommand()
  },

  created: function () {
    // Make sure local odom format matches vuex odom format
    // this.odom_format_in = this.odom_format

    auton_publish_interval = window.setInterval(() => {
      if (this.waitingForNavResponse) {
        this.sendAutonCommand()
      }
    }, 1000)
    window.setTimeout(() => {
      // Timeout so websocket will be initialized
      this.sendMessage({ type: 'get_auton_waypoint_list' })
    }, 250)
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    ...mapMutations('autonomy', {
      setRoute: 'setRoute',
      setWaypointList: 'setWaypointList',
      setAutonMode: 'setAutonMode',
      setTeleopMode: 'setTeleopMode'
    }),

    ...mapMutations('map', {
      setOdomFormat: 'setOdomFormat'
    }),

    sendAutonCommand() {
      if (this.autonEnabled) {
        this.sendMessage({
          type: 'auton_enable',
          enabled: true,
          waypoints: _.map(
            this.route,
            (waypoint) => {
              const lat = waypoint.lat
              const lon = waypoint.lon
              // Return a GPSWaypoint.msg formatted object for each
              return {
                latitude_degrees: lat,
                longitude_degrees: lon,
                tag_id: waypoint.id,
                type: waypoint.type
              }
            }
          )
        })
      } else {
        //if auton's not enabled, send an empty message
        this.sendMessage({ type: 'auton_enable', enabled: false, waypoints: [] })
      }
    },

    deleteItem: function (waypoint) {
      waypoint.in_route = false
      let index = this.route.indexOf(waypoint)
      this.route.splice(index, 1)
    },

    // Add item from all waypoints div to current waypoints div
    addItem: function (waypoint) {
      if (!waypoint.in_route) {
        this.route.push(waypoint)
        waypoint.in_route = true
      }
    },

    openModal: function() {
      this.modal.show()
    },

    addMapWaypoint: function() {
      this.modalWypt.lat = this.clickPoint.lat;
      this.modalWypt.lon = this.clickPoint.lon;
      this.waypoints.push(this.modalWypt);
      this.modalWypt = {
        name: '',
        id: -1,
        type: 0,
        lat: 0,
        lon: 0,
      }
      this.modal.hide()
    },

    deleteMapWaypoint: function(index:number) {
      this.waypoints.splice(index, 1)
    },

    toggleAutonMode: function (val: boolean) {
      this.setAutonMode(val)
      // This will trigger the yellow "waiting for nav" state of the checkbox
      this.autonButtonColor = 'btn-warning'
      this.waitingForNavResponse = true
    },

    toggleTeleopMode: function () {
      this.teleopEnabledCheck = !this.teleopEnabledCheck
      this.sendMessage({ type: 'teleop_enable', enabled: this.teleopEnabledCheck })
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
  height: 90%;
  overflow-y: auto;
}

.route {
  height: 60%;
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
