<template>
    <div class="wrapper">
        <div class="shadow p-3 mb-5 header">
            <img class="logo" src="mrover.png" alt="MRover" title="MRover" width="200" />
            <h1>Auton Dashboard</h1>
            <!-- <MCUReset class="mcu_reset"></MCUReset>
        <CommReadout class="comms"></CommReadout> -->
            <div class="help">
                <img src="help.png" alt="Help" title="Help" width="48" height="48" />
            </div>
            <div class="helpscreen"></div>
            <div class="helpimages" style="
                display: flex;
                align-items: center;
                justify-content: space-evenly;
                ">
                <img src="joystick.png" alt="Joystick" title="Joystick Controls"
                    style="width: auto; height: 70%; display: inline-block" />
            </div>
        </div>
        <div class="shadow p-3 rounded data" :style="`background-color: {{nav_state_color}}`">
            <div>
                <h2>Nav State: {{ nav_status.nav_state_name }}</h2>
                <CameraFeed></CameraFeed>
            </div>
            <div>
            <p style="margin-top: 6px">Joystick Values</p>
            </div>
            <!-- <JoystickValues /> -->
            <OdometryReading :odom="odom"></OdometryReading>
        </div>
        <div class="shadow p-3 rounded map">
            <AutonRoverMap :odom="odom" />
        </div>
        <div class="shadow p-3 rounded waypoints">
        <AutonWaypointEditor
        :odom="odom"
        @toggleTeleop="teleopEnabledCheck = $event"
        />
        </div>
        <!--Enable the drive controls if auton is off-->
        <!-- <div
        v-if="!autonEnabled && teleopEnabledCheck"
        v-show="false"
        class="driveControls"
    >
        <DriveControls />
    </div> -->
        <!-- <div v-show="false">
        <MastGimbalControls></MastGimbalControls>
    </div> -->
        <div class="conditions">
            <div v-if="!stuck_status" class="shadow p-3 rounded stuck not-stuck">
                <h4>Nominal Conditions</h4>
            </div>
            <div v-else class="shadow p-3 rounded stuck rover-stuck">
                <h4>Obstruction Detected</h4>
            </div>
        </div>
        <div class="shadow p-3 rounded cameras">
        <Cameras :primary="true" />
        </div>
        <div class="shadow p-3 rounded moteus">
            <DriveMoteusStateTable :moteus-state-data="moteusState" />
            <JointStateTable :joint-state-data="jointState" :vertical="true" />
        </div>
    </div>
</template>
  
<script lang="ts">
// import { mapGetters } from "vuex";
import DriveMoteusStateTable from "./DriveMoteusStateTable.vue";
import AutonRoverMap from "./AutonRoverMap.vue";
import AutonWaypointEditor from "./AutonWaypointEditor.vue";
import CameraFeed from "./CameraFeed.vue";
import Cameras from "./Cameras.vue";
import JointStateTable from "./JointStateTable.vue";
import OdometryReading from "./OdometryReading.vue";
// import { quaternionToMapAngle } from "../utils.js";
import { defineComponent } from "vue";

const navBlue: string = "#4695FF";
const navGreen: string = "yellowgreen";
const navRed: string = "lightcoral";
const navGrey: string = "lightgrey";

const ledUpdateRate: number = 1;
let ledInterval: number;

export default defineComponent({
    components: {
        DriveMoteusStateTable,
        AutonRoverMap,
        AutonWaypointEditor,
        CameraFeed,
        Cameras,
        JointStateTable,
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

            // Default object isn't empty, so has to be initialized to ""
            moteusState: {
                name: ["", "", "", "", "", ""],
                error: ["", "", "", "", "", ""],
                state: ["", "", "", "", "", ""],
            },

            jointState: {},
        };
    },

    computed: {
        // ...mapGetters("autonomy", {
        //     autonEnabled: "autonEnabled",
        //     teleopEnabled: "teleopEnabled"
        // }),

        nav_state_color(): string {
            // if (!this.autonEnabled && this.teleopEnabledCheck) {
            //     return navBlue;
            // }
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
                //   this.sendColor();
            }
        }
    },

    beforeUnmount: function () {
        this.ledColor = "off";
        //   this.sendColor();
        window.clearInterval(ledInterval);
    },

    created: function () {
        // Blink interval for green and off flasing
        setInterval(() => {
            this.navBlink = !this.navBlink;
        }, 500);

        // Initialize color to red.
        this.ledColor = "red";
        //   this.sendColor();

        ledInterval = window.setInterval(() => {
            // this.sendColor();
        }, ledUpdateRate * 1000);
    },

    methods: {
    }
});
</script>
  
<style scoped>
.wrapper {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: 45vw 15vw auto;
    grid-template-rows: auto 50vh auto auto auto;
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


.stuck {
    grid-area: stuck;
    border-radius: 5px;
    line-height: 40px;
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


.header {
  grid-area: header;
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px;
}

.logo {
  position: absolute;
  left: 50%;
  transform: translateX(-50%);
}

h2 {
    padding: 2px;
    margin: 0px;
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

.help:hover~.helpscreen,
.help:hover~.helpimages {
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