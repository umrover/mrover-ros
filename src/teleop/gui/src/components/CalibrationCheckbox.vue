<template>
    <div class="calibration-wrapper">
        <Checkbox
        :name="name"        
        @toggle="toggleCalibration">
        </Checkbox>
        <span
            :class="[
              'led',
              enabled ? 'green' : 'red',
            ]"
        ></span>
    </div>
</template>

<script>
import Checkbox from "./Checkbox.vue";
import ROSLIB from "roslib/src/RosLib";

export default {
    props: {
        name: {
            type: String,
            required: true
        }
    },

    data() {
        return {
            enabled: false,
            calibrate_service: null,
            calibrate_sub: null
        }
    },
    
    components: {
        Checkbox
    },
    
    created: function () {
        this.calibrate_service = new ROSLIB.Service({
            ros: this.$ros,
            name: "calibrate",
            serviceType: "mrover/CalibrateMotors"
        });

        this.calibrate_sub = new ROSLIB.Topic({
            ros: this.$ros,
            name: "ra_is_calibrated",
            messageType: "mrover/Calibrated"
        });

        this.calibrate_sub.subscribe(
            (msg) => {
                console.log(msg.names)
                var index = 0;
                for (var i = 0; i < msg.names.length; ++i) {
                    if ((msg.names[i] == "sa_joint_1" && 
                        this.name == "Joint 1 Calibration") ||
                        (msg.names[i] == "sa_joint_2" && 
                        this.name == "Joint 2 Calibration") ||
                        (msg.names[i] == "sa_joint_3" && 
                        this.name == "Joint 3 Calibration") 
                        ) {
                        index = i;
                    }
                }

                this.enabled = msg.calibrated[index];
            }
        );
    },

    methods: {
        toggleCalibration: function () {
            this.enabled = !this.enabled;
            let request = new ROSLIB.ServiceRequest({
                name: this.name,
                calibrate: this.enabled
            });
            this.calibrate_service.callService(request, (result) => {
                if (!result) {
                    this.enabled = !this.enabled;
                    alert("Toggling Calibration failed.");
                }
            });
        }
    }
}

</script>

<style>
    .calibration-wrapper {
        padding: 2%;
        display: flex;
        flex-direction: row;
    }
    .led {
        margin-left: 5%;
        margin-top: 1%;
        width: 16px;
        height: 16px;
        border-radius: 8px;
        border: 1px solid;
        display: block;
    }
    .green {
        background-color: lightgreen;
    }
    .red {
        background-color: red;
    }
</style>