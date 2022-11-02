<template>
    <div class="wrap">
        <l-map ref="map" class="map" :zoom="15" :center="center">
        <l-control-scale :imperial="false"/>
        <l-tile-layer ref="tileLayer" :url="this.online ? this.onlineUrl : this.offlineUrl" :attribution="attribution" 
                      :options="this.online ? this.onlineTileOptions : this.offlineTileOptions"/>
        <l-marker ref="rover" :lat-lng="odomLatLng" :icon="locationIcon"/>

        <div v-for="(waypoint, index) in waypointList" :key="index">
            <div v-if="index===highlightedWaypoint">
                <l-marker :lat-lng="waypoint.latLng" :icon="highlightedWaypointIcon">
                <l-tooltip :options="{permanent: 'true', direction: 'top'}"> {{ waypoint.name }}, {{ index }} </l-tooltip>
                </l-marker>
            </div>
            <div v-else>
                <l-marker :lat-lng="waypoint.latLng" :icon="waypointIcon">
                <l-tooltip :options="{permanent: 'true', direction: 'top'}"> {{ waypoint.name }}, {{ index }} </l-tooltip>
                </l-marker>
            </div>
        </div>

        <l-polyline :lat-lngs="odomPath" :color="'blue'"/>
        </l-map>
        <div class="controls">
            <div class="online">
                <label><input type="checkbox" v-model="online" />Online</label>
            </div> 
        </div>
    </div>
</template>
  
<script>
    import { LMap, LTileLayer, LMarker, LPolyline, LPopup, LTooltip, LControlScale } from 'vue2-leaflet'
    import { mapGetters } from 'vuex'
    import L from '../leaflet-rotatedmarker.js'

    const MAX_ODOM_COUNT = 1000
    const DRAW_FREQUENCY = 10
    const onlineUrl = 'http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'
    const offlineUrl = '/static/map/{z}/{x}/{y}.png'
    const onlineTileOptions = {
        maxNativeZoom: 22,
        maxZoom: 100,
        subdomains: ['mt0','mt1','mt2','mt3'] 
    }
    const offlineTileOptions = {
            maxNativeZoom: 22,
            maxZoom: 100,
    }

    export default {
        name: 'RoverMap',

        data () {
            return {
                // Default Center In NC 53 Parking Lot
                center: L.latLng(42.294864932393835, -83.70781314674628,),
                attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
                online: true,
                onlineUrl: onlineUrl,
                offlineUrl: offlineUrl,
                onlineTileOptions: onlineTileOptions,
                offlineTileOptions: offlineTileOptions,
                roverMarker: null,
                waypointIcon: null,
                highlightedWaypointIcon: null,
                map: null,
                odomCount: 0,
                locationIcon: null,
                odomPath: [],
                findRover: false
            }
        },

        components: {
            LMap,
            LTileLayer,
            LMarker,
            LPolyline,
            LPopup,
            LTooltip,
            LControlScale
        },

        created: function () {
            this.locationIcon = L.icon({
                iconUrl: '/static/location_marker_icon.png',
                iconSize: [64, 64],
                iconAnchor: [32, 32]
            })
            this.waypointIcon = L.icon({
                iconUrl: '/static/map_marker.png',
                iconSize: [64, 64],
                iconAnchor: [32, 64],
                popupAnchor: [0, -32]
            })
            this.highlightedWaypointIcon = L.icon({
                iconUrl: '/static/map_marker_highlighted.png',
                iconSize: [64, 64],
                iconAnchor: [32, 64],
                popupAnchor: [0, -32]
            })
        },

        computed: {
            ...mapGetters('erd', {
                waypointList: 'waypointList',
                highlightedWaypoint: "highlightedWaypoint",
            }),

            odomLatLng: function () {
                return L.latLng(this.odom.latitude_deg + this.odom.latitude_min/60, this.odom.longitude_deg + this.odom.longitude_min/60)
            },
            polylinePath: function () {
                return [this.odomLatLng].concat(this.route.map(waypoint => waypoint.latLng))
            }
        },

        props: {
            odom: {
                type: Object,
                required: true
            }
        },

        watch: {
            odom: function (val) {
                // Trigger every time rover odom is changed
                const lat = val.latitude_deg + val.latitude_min / 60
                const lng = val.longitude_deg + val.longitude_min / 60
                const angle = val.bearing_deg
                // Move to rover on first odom message
                if (!this.findRover) {
                    this.findRover = true
                    this.center = L.latLng(lat, lng)
                }
                // Update the rover marker
                this.roverMarker.setRotationAngle(angle)
                this.roverMarker.setLatLng(L.latLng(lat, lng))
                // Update the rover path
                this.odomCount++
                if (this.odomCount % DRAW_FREQUENCY === 0) {
                    if (this.odomCount > MAX_ODOM_COUNT * DRAW_FREQUENCY) {
                        this.odomPath.splice(0, 1)
                    }
                    this.odomPath.push(L.latLng(lat, lng))
                }
                this.odomPath[this.odomPath.length - 1] = L.latLng(lat, lng)
            }
        },
        mounted: function () {
            this.$nextTick(() => {
                this.map = this.$refs.map.mapObject
                this.roverMarker = this.$refs.rover.mapObject
            })
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
    }
</style>