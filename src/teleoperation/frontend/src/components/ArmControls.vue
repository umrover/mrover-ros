<template>
    <div class="wrap">
        <h2>Arm Controls</h2>
        <div class="controls-flex">
            <h4>Arm mode</h4>
            <!-- Make opposite option disappear so that we cannot select both -->
            <!-- Change to radio buttons in the future -->
            <input ref="arm-enabled" v-model="arm_mode" type="radio" :name="'Arm Enabled'" value="arm_disabled" />
            Arm Disabled
            <input ref="open-loop-enabled" v-model="arm_mode" type="radio" :name="'Open Loop Enabled'" value="open_loop" />
            IK
            <input ref="arm-enabled" v-model="arm_mode" type="radio" :name="'Arm Enabled'" value="arm_disabled" />
            Position
            <input ref="open-loop-enabled" v-model="arm_mode" type="radio" :name="'Open Loop Enabled'" value="open_loop" />
            Velocity
            <input ref="arm-enabled" v-model="arm_mode" type="radio" :name="'Arm Enabled'" value="arm_disabled" />
            Throttle
            <!-- Commented until servoing works :( -->
            <!-- <input
              ref="servo-enabled"
              v-model="arm_mode"
              type="radio"
              :name="'Servo'"
              value="servo"
            />
            Servo -->
        </div>
        <!-- Commented until joint locking is implemented -->
        <!-- <div class="controls-flex">
            <h4>Joint Locks</h4>
            <Checkbox ref="A" :name="'A'" @toggle="updateJointsEnabled(0, $event)" />
            <Checkbox ref="B" :name="'B'" @toggle="updateJointsEnabled(1, $event)" />
            <Checkbox ref="C" :name="'C'" @toggle="updateJointsEnabled(2, $event)" />
            <Checkbox ref="D" :name="'D'" @toggle="updateJointsEnabled(3, $event)" />
            <Checkbox ref="E" :name="'E'" @toggle="updateJointsEnabled(4, $event)" />
            <Checkbox ref="F" :name="'F'" @toggle="updateJointsEnabled(5, $event)" />
          </div> -->
        <div class="controls-flex">
            <h4>Misc. Controls</h4>
            <ToggleButton id="arm_laser" :current-state="laser_enabled" label-enable-text="Arm Laser On"
                label-disable-text="Arm Laser Off" @change="toggleArmLaser()" />
            <div class="limit-switch">
                <h4>Limit Switches</h4>
                <LimitSwitch :switch_name="'joint_b'" :name="'All Switches'" />
            </div>
        </div>
        <div class="controls-flex">
            <h4>Calibration</h4>
            <CalibrationCheckbox name="All Joints Calibration" joint_name="joint_b" calibrate_topic="ra_is_calibrated" />
            <JointAdjust :options="[
                { name: 'joint_a', option: 'Joint A' },
                { name: 'joint_b', option: 'Joint B' },
                { name: 'joint_c', option: 'Joint C' },
                { name: 'joint_d', option: 'Joint D' },
                { name: 'joint_e', option: 'Joint E' }
            ]" />
        </div>
    </div>
</template>
  
<script lang ="ts">
import { inject, defineComponent } from 'vue';
import ToggleButton from "./ToggleButton.vue";
import CalibrationCheckbox from "./CalibrationCheckbox.vue";
import JointAdjust from "./MotorAdjust.vue";
import LimitSwitch from "./LimitSwitch.vue";

import { mapState, mapActions } from 'vuex';

// In seconds
const updateRate = 0.1;
let interval;

export default defineComponent({

    components: {
        ToggleButton,
        CalibrationCheckbox,
        JointAdjust,
        LimitSwitch
    },
    data() {
        return {
            // websocket: inject("webSocketService") as WebSocket,
            // websocket: new WebSocket('ws://localhost:8000/ws/gui'),
            arm_mode: "arm_disabled",
            joints_array: [false, false, false, false, false, false],
            laser_enabled: false,
            ra_mode_service: null,
            jointlock_pub: null,
            joystick_pub: null,
            laser_service: null,
        };
    },

    computed: {
        ...mapState('websocket', ['message'])
    },

    watch: {
        message(msg) {
            if(msg.type=="laser_service"){
                if (!msg.result) {
                    this.laser_enabled = !this.laser_enabled;
                    alert("Toggling Arm Laser failed.");
                }
            }
        }
    },

    // watch: {
    //     arm_mode: function (newMode, oldMode) {
    //         this.updateArmMode(newMode, oldMode);
    //     }
    // },

    // beforeDestroy: function () {
    //     this.updateArmMode("arm_disabled", this.arm_mode);
    //     window.clearInterval(interval);
    // },

    // created: function () {
    //     this.websocket.onmessage = (event) => { console.log(event.data)
    //         const msg = JSON.parse(event.data);
    //         if(msg.type=="laser_service"){
    //             if (!msg.result) {
    //                 this.laser_enabled = !this.laser_enabled;
    //                 alert("Toggling Arm Laser failed.");
    //             }
    //         }
    //     };
    // },

    // created: function () {
    //     this.joystick_pub = new ROSLIB.Topic({
    //         ros: this.$ros,
    //         name: "/xbox/ra_control",
    //         messageType: "sensor_msgs/Joy"
    //     });
   
    //     this.ra_mode_service = new ROSLIB.Service({
    //         ros: this.$ros,
    //         name: "change_ra_mode",
    //         serviceType: "mrover/ChangeArmMode"
    //     });
    //     this.jointlock_pub = new ROSLIB.Topic({
    //         ros: this.$ros,
    //         name: "/joint_lock",
    //         messageType: "mrover/JointLock"
    //     });
    //     this.updateArmMode("arm_disabled", this.arm_mode);
    //     const jointData = {
    //         //publishes array of all falses when refreshing the page
    //         joints: this.joints_array
    //     };
    //     var jointlockMsg = new ROSLIB.Message(jointData);
    //     this.jointlock_pub.publish(jointlockMsg);

    //     interval = window.setInterval(() => {
    //         const gamepads = navigator.getGamepads();
    //         for (let i = 0; i < 4; i++) {
    //             const gamepad = gamepads[i];
    //             if (gamepad) {
    //                 // Microsoft and Xbox for old Xbox 360 controllers
    //                 // X-Box for new PowerA Xbox One controllers
    //                 if (
    //                     gamepad.id.includes("Microsoft") ||
    //                     gamepad.id.includes("Xbox") ||
    //                     gamepad.id.includes("X-Box")
    //                 ) {
    //                     let buttons = gamepad.buttons.map((button) => {
    //                         return button.value;
    //                     });
    //                     this.publishJoystickMessage(gamepad.axes, buttons);
    //                 }
    //             }
    //         }
    //     }, updateRate * 1000);
    // },

    methods: {
    //     updateArmMode: function (newMode, oldMode) {
    //         const armData = {
    //             mode: newMode
    //         };
    //         var armcontrolsmsg = new ROSLIB.ServiceRequest(armData);
    //         this.ra_mode_service.callService(armcontrolsmsg, (response) => {
    //             if (!response.success) {
    //                 this.arm_mode = oldMode;
    //                 alert("Failed to change arm mode");
    //             }
    //         });
    //     },

    //     updateJointsEnabled: function (jointnum, enabled) {
    //         this.joints_array[jointnum] = enabled;
    //         const jointData = {
    //             joints: this.joints_array
    //         };
    //         var jointlockMsg = new ROSLIB.Message(jointData);
    //         this.jointlock_pub.publish(jointlockMsg);
    //     },

    //     publishJoystickMessage: function (axes, buttons) {
    //         const joystickData = {
    //             axes: axes,
    //             buttons: buttons
    //         };
    //         var joystickMsg = new ROSLIB.Message(joystickData);
    //         this.joystick_pub.publish(joystickMsg);
    //     },
    ...mapActions('websocket', ['sendMessage']),
    
        toggleArmLaser: function () {
            this.laser_enabled = !this.laser_enabled;
            this.sendMessage({type:"laser_service", data:this.laser_enabled});
            
         }
    }
});
</script>
  
<style scoped>
.wrap {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-items: center;
    width: 100%;
    height: auto;
}

.wrap h2 h4 {
    margin: 0;
    padding: 0;
    font-size: 1.5em;
    font-weight: bold;
    text-align: center;
    width: 100%;
    padding-top: 5px;
    padding-bottom: 5px;
}

.controls-flex {
    display: flex;
    align-items: center;
    width: 100%;
    column-gap: 20px;
    padding-left: 10px;
    margin-bottom: 5px;
    margin-top: 5px;
}

.limit-switch {
    display: flex;
    flex-direction: column;
    align-items: center;
}

.limit-switch h4 {
    margin-bottom: 5px;
}
</style>