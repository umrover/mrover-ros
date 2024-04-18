<template>
  <div class="wrap">
      <l-map @ready="onMapReady" ref="map" class="map" :zoom="22" :center="center" @click="getClickedLatLon($event)">
          <l-control-scale :imperial="false" />
          <l-tile-layer ref="tileLayer" :url="online ? onlineUrl : offlineUrl" :attribution="attribution" :options="online ? onlineTileOptions : offlineTileOptions" />
  
          <l-marker ref="rover" :lat-lng="odomLatLng" :icon="locationIcon" />
          <l-marker ref="drone" :lat-lng="droneLatLng" :icon="droneIcon" />
  
          <div v-for="(waypoint, index) in waypointList" :key="index">
              <l-marker :lat-lng="waypoint.latLng" :icon="getWaypointIcon(waypoint, index)">
                  <l-tooltip :options="{ permanent: 'true', direction: 'top' }">
                      {{ waypoint.name }}, {{ index }}
                  </l-tooltip>
              </l-marker>
          </div>
  
          <l-polyline :lat-lngs="odomPath" :color="'blue'" />
          <l-polyline :lat-lngs="dronePath" :color="'green'" />
      </l-map>
      <label><input v-model="online" type="checkbox" />Online</label>
  </div>
  </template>
  
  <script lang="ts">
  import {
      LMap,
      LTileLayer,
      LMarker,
      LPolyline,
      LTooltip,
      LControlScale
  } from '@vue-leaflet/vue-leaflet'
  import {
      mapGetters,
      mapMutations,
      mapActions,
      mapState
  } from 'vuex'
  import 'leaflet/dist/leaflet.css'
  import L from '../leaflet-rotatedmarker.js'
  
  const MAX_ODOM_COUNT = 1000
  const DRAW_FREQUENCY = 1
  const onlineUrl = 'http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'
  const offlineUrl = 'map/{z}/{x}/{y}.png'
  const onlineTileOptions = {
      maxNativeZoom: 22,
      maxZoom: 100,
      subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
  }
  const offlineTileOptions = {
      maxNativeZoom: 16,
      maxZoom: 100
  }
  
  export default {
      name: 'RoverMap',
  
      components: {
          LMap,
          LTileLayer,
          LMarker,
          LPolyline,
          LTooltip,
          LControlScale
      },
  
      data() {
          return {
              // Default Center at MDRS
              center: L.latLng(38.406025, -110.7923723),
              attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
              online: true,
              onlineUrl: onlineUrl,
              offlineUrl: offlineUrl,
              onlineTileOptions: onlineTileOptions,
              offlineTileOptions: offlineTileOptions,
              roverMarker: null,
              droneMarker: null,
              waypointIcon: null,
              highlightedWaypointIcon: null,
              map: null,
              odomCount: 0,
              droneCount: 0,
              locationIcon: null,
              droneIcon: null,
              odomPath: [],
              dronePath: [],
              findRover: false,
              drone_latitude_deg:42.293195,
              drone_longitude_deg:-83.7096706,
              circle: null, //search radius
          }
      },
  
      created: function () {
          this.locationIcon = L.icon({
              iconUrl: '/location_marker_icon.png',
              iconSize: [64, 64],
              iconAnchor: [32, 32]
          })
          this.droneIcon = L.icon({
              iconUrl: '/drone_icon_1.png',
              iconSize: [64, 64],
              iconAnchor: [32, 32]
          })
          this.waypointIcon = L.icon({
              iconUrl: '/map_marker.png',
              iconSize: [64, 64],
              iconAnchor: [32, 64],
              popupAnchor: [0, -32]
          })
          this.droneWaypointIcon = L.icon({
              iconUrl: '/map_marker_drone.png',
              iconSize: [64, 64],
              iconAnchor: [32, 64],
              popupAnchor: [0, -32]
          })
          this.highlightedWaypointIcon = L.icon({
              iconUrl: '/map_marker_highlighted.png',
              iconSize: [64, 64],
              iconAnchor: [32, 64],
              popupAnchor: [0, -32]
          })
      },
  
      methods: {
        ...mapActions('websocket', ['sendMessage']),
          onMapReady: function () {
              // Pull objects from refs to be able to access data and change w functions
              this.$nextTick(() => {
                  this.map = this.$refs.map.leafletObject
                  this.roverMarker = this.$refs.rover.leafletObject
                  this.droneMarker = this.$refs.drone.leafletObject
              })
          },
          // Event listener for setting store values to get data to waypoint Editor
          getClickedLatLon: function (e: any) {
              this.setClickPoint({
                  lat: e.latlng.lat,
                  lon: e.latlng.lng
              })
          },
          getWaypointIcon: function (waypoint: any, index: number) {
              if (index === this.highlightedWaypoint) {
                  return this.highlightedWaypointIcon
              } else if (waypoint.drone) {
                  return this.droneWaypointIcon
              } else {
                  return this.waypointIcon
              }
          },
          ...mapMutations('erd', {
              setClickPoint: 'setClickPoint',
              setWaypointList: 'setWaypointList',
              setOdomFormat: 'setOdomFormat'
          })
      },
  
      computed: {
         ...mapState('websocket', ['message']),

          ...mapGetters('erd', {
              waypointList: 'waypointList',
              highlightedWaypoint: 'highlightedWaypoint',
              searchWaypoint: 'searchWaypoint'
          }),
  
          // Convert to latLng object for Leaflet to use
          odomLatLng: function () {
              return L.latLng(this.odom.latitude_deg, this.odom.longitude_deg)
          },

          droneLatLng: function () {
              return L.latLng(this.drone_latitude_deg, this.drone_longitude_deg)
          },
  
          polylinePath: function () {
              return [this.odomLatLng].concat(this.route.map((waypoint: any) => waypoint.latLng))
          },

          dronepolylinePath: function () {
              return [this.droneLatLng].concat(this.route.map((waypoint: any) => waypoint.latLng))
          },
      },
  
      props: {
          odom: {
              type: Object,
              required: true
          }
      },
  
      watch: {
          message(msg) {
            if (msg.type == 'drone_waypoint') {
                this.drone_latitude_deg = msg.latitude
                this.drone_longitude_deg = msg.longitude
                const latLng = L.latLng(this.drone_latitude_deg, this.drone_longitude_deg)
                this.droneMarker.setLatLng(latLng)
                  // Update the rover path
                  this.droneCount++
                  if (this.droneCount % DRAW_FREQUENCY === 0) {
                      if (this.dronePath.length > MAX_ODOM_COUNT) {
                          this.dronePath = [...this.dronePath.slice(1), latLng] //remove oldest element
                      }
  
                      this.dronePath = [...this.dronePath, latLng]
                      this.droneCount = 0
                  }
            }
          },

          odom: {
              handler: function (val) {
                  // Trigger every time rover odom is changed
  
                  const lat = val.latitude_deg
                  const lng = val.longitude_deg
                  const angle = val.bearing_deg
  
                  const latLng = L.latLng(lat, lng)
  
                  // Move to rover on first odom message
                  if (!this.findRover) {
                      this.findRover = true
                      this.center = latLng
                  }
  
                  // Update the rover marker using bearing angle
                  this.roverMarker.setRotationAngle(angle)
  
                  this.roverMarker.setLatLng(latLng)
  
                  // Update the rover path
                  this.odomCount++
                  if (this.odomCount % DRAW_FREQUENCY === 0) {
                      if (this.odomPath.length > MAX_ODOM_COUNT) {
                          this.odomPath = [...this.odomPath.slice(1), latLng] //remove oldest element
                      }
  
                      this.odomPath = [...this.odomPath, latLng]
                      this.odomCount = 0
                  }
              },
              // Deep will watch for changes in children of an object
              deep: true
          },

          searchWaypoint(newIndex) {
            let waypoint = this.waypointList[newIndex];
            if(!this.circle) this.circle = L.circle(waypoint.latLng, {radius: 200}).addTo(this.map);
            else this.circle.setLatLng(waypoint.latLng)
            this.circle.setStyle({fillColor: 'purple', stroke: false})
          }
      }
  }
  </script>
  
  <style scoped>
  .map {
      height: 100%;
      width: 100%;
  }
  
  .wrap {
      display: flex;
      align-items: center;
      height: 100%;
      width: 100%;
  }
  </style>
  