<template>
    <div class="wrap">
        <h2>Arm Controls</h2>
        <div class="controls-flex">
            <h4>Arm mode</h4>
            <!-- Make opposite option disappear so that we cannot select both -->
            <!-- Change to radio buttons in the future -->
            <input ref="arm-enabled" v-model="arm_mode" type="radio" name="'arm_mode'" value="arm_disabled" />
            Arm Disabled
            <input ref="ik" v-model="arm_mode" type="radio" name="'arm_mode'" value="ik" />
            Open Loop
            <input ref="position" v-model="arm_mode" type="radio" name="'arm_mode'" value="position" />
            Position
            <input ref="velocity" v-model="arm_mode" type="radio" name="'arm_mode'" value="velocity" />
            Velocity
            <input ref="throttle" v-model="arm_mode" type="radio" name="'arm_mode'" value="throttle" />
            Throttle
           
        </div>
       
        <div class="controls-flex">
            <h4>Misc. Controls</h4>
            <ToggleButton id="arm_laser" :current-state="laser_enabled" label-enable-text="Arm Laser On"
                label-disable-text="Arm Laser Off" @change="toggleArmLaser()" />
            <div class="limit-switch">
                <h4>Joint B Limit Switch</h4>
                <LimitSwitch :name="'All Switches'" />
            </div>
        </div>
        <div class="controls-flex">
            <h4>Calibration</h4>
            <CalibrationCheckbox name="All Joints Calibration"/>
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

// In seconds
const updateRate = 0.1;
let interval: number | undefined;

export default defineComponent({
    components: {
        ToggleButton,
        CalibrationCheckbox,
        JointAdjust,
        LimitSwitch
    },
    data() {
        return {
            websocket: new WebSocket("ws://localhost:8000/ws/gui"),
            arm_mode: "arm_disabled",
            joints_array: [false, false, false, false, false, false],
            laser_enabled: false
        };
    },

    // watch: {
    //     arm_mode: function (newMode, oldMode) {
    //         this.updateArmMode(newMode, oldMode);
    //     }
    // },

    // beforeUnmount: function () {
    //     this.updateArmMode("arm_disabled", this.arm_mode);
    //     window.clearInterval(interval);
    // },

    created: function () {
        this.websocket.onmessage = (event) => {
            const msg = JSON.parse(event.data);
            if(msg.type=="laser_service"){
                if (!msg.result) {
                    this.laser_enabled = !this.laser_enabled;
                    alert("Toggling Arm Laser failed.");
                }
            }
        };
        interval = window.setInterval(() => {
            const gamepads = navigator.getGamepads();
            for (let i = 0; i < 4; i++) {
                const gamepad = gamepads[i];
                if (gamepad) {
                    // Microsoft and Xbox for old Xbox 360 controllers
                    // X-Box for new PowerA Xbox One controllers
                    if (
                        gamepad.id.includes("Microsoft") ||
                        gamepad.id.includes("Xbox") ||
                        gamepad.id.includes("X-Box")
                    ) {
                        let buttons = gamepad.buttons.map((button) => {
                            return button.value;
                        });
                        this.publishJoystickMessage(gamepad.axes, buttons, this.arm_mode);
                    }
                }
            }
        }, updateRate * 1000);
        // this.updateArmMode("arm_disabled", this.arm_mode);
        // const jointData = {
        // //publishes array of all falses when refreshing the page
        // joints: this.joints_array
        // };
    },

    
   
        // this.ra_mode_service = new ROSLIB.Service({
        //     ros: this.$ros,
        //     name: "change_ra_mode",
        //     serviceType: "mrover/ChangeArmMode"
        // });
    //     this.jointlock_pub = new ROSLIB.Topic({
    //         ros: this.$ros,
    //         name: "/joint_lock",
    //         messageType: "mrover/JointLock"
    //     });
    //     const jointData = {
    //         //publishes array of all falses when refreshing the page
    //         joints: this.joints_array
    //     };
    //     var jointlockMsg = new ROSLIB.Message(jointData);
    //     this.jointlock_pub.publish(jointlockMsg);

 
    // },

    methods: {
        // updateArmMode: function (newMode: any, oldMode: string) {
        //     const armData = {
        //         mode: newMode
        //     };

        //     var armcontrolsmsg = new ROSLIB.ServiceRequest(armData);
        //     this.ra_mode_service.callService(armcontrolsmsg, (response: { success: any; }) => {
        //         if (!response.success) {
        //             this.arm_mode = oldMode;
        //             alert("Failed to change arm mode");
        //         }
        //     });
        // },

    //     updateJointsEnabled: function (jointnum, enabled) {
    //         this.joints_array[jointnum] = enabled;
    //         const jointData = {
    //             joints: this.joints_array
    //         };
    //         var jointlockMsg = new ROSLIB.Message(jointData);
    //         this.jointlock_pub.publish(jointlockMsg);
    //     },

        publishJoystickMessage: function (axes: any, buttons: any, arm_mode: any) {
            const joystickData = {
                axes: axes,
                buttons: buttons
            };
            this.websocket.send(JSON.stringify({type:"arm_values", data:joystickData, arm_mode: arm_mode}))
        },
        toggleArmLaser: function () {
            this.laser_enabled = !this.laser_enabled;
            this.websocket.send(JSON.stringify({type:"laser_service", data:this.laser_enabled}))
            
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