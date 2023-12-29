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
        <div :class="['shadow p-3 rounded data', ledColor]">
            <h2>Nav State: {{ navState }}</h2>
            <div style="display: inline-block;">
                <CameraFeed></CameraFeed>
            </div>
            <div style="display: inline-block; vertical-align: top;">
            <p style="margin-top: 6px">Joystick Values</p>
            <JoystickValues />
            </div>
            <OdometryReading :odom="odom" />
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
        <div
        v-if="!autonEnabled && teleopEnabledCheck"
        v-show="false"
        class="driveControls"
    >
        <DriveControls />
        </div>
        <!-- <div v-show="false">
        <MastGimbalControls></MastGimbalControls>
    </div> -->
        <div class="conditions">
            <div v-if="!stuck_status" class="shadow p-3 rounded bg-success text-center">
                <h4>Nominal Conditions</h4>
            </div>
            <div v-else class="shadow p-3 rounded bg-danger text-center">
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
import { mapState, mapGetters } from "vuex";
import DriveMoteusStateTable from "./DriveMoteusStateTable.vue";
import AutonRoverMap from "./AutonRoverMap.vue";
import AutonWaypointEditor from "./AutonWaypointEditor.vue";
import CameraFeed from "./CameraFeed.vue";
import Cameras from "./Cameras.vue";
import JointStateTable from "./JointStateTable.vue";
import OdometryReading from "./OdometryReading.vue";
import JoystickValues from './JoystickValues.vue';
import DriveControls from "./DriveControls.vue";
import { quaternionToMapAngle } from "../utils.js";
import { defineComponent } from "vue";

let interval:number;

export default defineComponent({
    components: {
        DriveMoteusStateTable,
        AutonRoverMap,
        AutonWaypointEditor,
        CameraFeed,
        Cameras,
        JointStateTable,
        OdometryReading,
        JoystickValues,
        DriveControls
    },

    data() {
        return {
            // Default coordinates are at MDRS
            odom: {
                latitude_deg: 38.4060250,
                longitude_deg: -110.7923723,
                bearing_deg: 0
            },

            teleopEnabledCheck: false,

            ledColor: "bg-danger", //red

            stuck_status: false,

            navState: "OffState",
            
            moteusState: {
                name: [] as string[],
                error: [] as string[],
                state: [] as string[],
            },

            jointState: {
                name: [] as string[],
                position: [] as number[],
                velocity: [] as number[],
                effort: [] as number[]
            }
        };
    },

    computed: {
        ...mapState('websocket', ['message']),

        ...mapGetters("autonomy", {
            autonEnabled: "autonEnabled",
            teleopEnabled: "teleopEnabled"
        }),
    },

    watch: {
        message(msg) {
            if (msg.type == "joint_state") {
                this.jointState.name = msg.name;
                this.jointState.position = msg.position;
                this.jointState.velocity = msg.velocity;
                this.jointState.effort = msg.effort;
            }
            else if(msg.type == "drive_moteus") {
                let index = this.moteusState.name.findIndex((n) => n === msg.name);
                if(this.moteusState.name.length == 6 || index != -1) {
                    //if all motors are in table or there's an update to one before all are in
                    if (index !== -1) {
                        this.moteusState.state[index] = msg.state;
                        this.moteusState.error[index] = msg.error;
                    }
                    else {
                        console.log("Invalid arm moteus name: " + msg.name);
                    }
                }
                else {
                    this.moteusState.name.push(msg.name);
                    this.moteusState.state.push(msg.state);
                    this.moteusState.error.push(msg.error);
                }
            }
            else if(msg.type == "led") {
                if(msg.red) this.ledColor = "bg-danger"; //red
                else if(msg.green) this.ledColor = "blink"; //blinking green
                else if(msg.blue) this.ledColor = "bg-primary"; //blue
            }
            else if(msg.type == "nav_state") {
                this.navState = msg.state;
            }
            else if(msg.type == "nav_sat_fix") {
                this.odom.latitude_deg = msg.latitude;
                this.odom.longitude_deg = msg.longitude;
            }
            else if(msg.type == "auton_tfclient") {
                this.odom.bearing_deg = quaternionToMapAngle(msg.rotation);
            }
        }
    },

    beforeUnmount: function () {
        this.ledColor = "bg-white";
        window.clearInterval(interval);
    },

    created() {
        interval = setInterval(() => {
            this.$websocket.send({type: "auton_tfclient"});
        }, 1000);
    }
});
</script>
  
<style scoped>
.wrapper {
    display: grid;
    grid-gap: 10px;
    grid-template-columns: 40% 20% auto;
    grid-template-rows: auto 40vh auto auto auto;
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

.blink {
    animation: blinkAnimation 1s infinite; /* Blinks green every second */
}

@keyframes blinkAnimation {
  0%, 100% {
    background-color: var(--bs-success);
  }
  50% {
    background-color: var(--bs-white);
  }
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