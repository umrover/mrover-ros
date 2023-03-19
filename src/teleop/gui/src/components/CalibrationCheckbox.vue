<template>
    <div class="calibration-wrapper">
        <Checkbox
        :name="name"        
        @toggle="toggleCalibration">
        </Checkbox>
        <span class="led">
            <LEDIndicator :connected="calibrated" :name="name" :show_name="false"/>
        </span>
    </div>
</template>

<script>
import Checkbox from "./Checkbox.vue";
import LEDIndicator from "./LEDIndicator.vue";
import ROSLIB from "roslib/src/RosLib";

let interval;

export default {
    components: {
        Checkbox,
        LEDIndicator
    },

    props: {
        name: {
            type: String,
            required: true
        },
        joint_name: {
            type: String,
            required: true
        },
        calibrate_topic: {
            type: String,
            required: true
        }  
    },

    data() {
        return {
            toggleEnabled: false,
            calibrated: false,
            calibrate_service: null,
            calibrate_sub: null
        }
    },

    beforeDestroy: function () {
        window.clearInterval(interval)
    },
    
    created: function () {
        this.calibrate_service = new ROSLIB.Service({
            ros: this.$ros,
            name: "calibrate",
            serviceType: "mrover/CalibrateMotors"
        });

        this.calibrate_sub = new ROSLIB.Topic({
            ros: this.$ros,
            name: this.calibrate_topic,
            messageType: "mrover/Calibrated"
        });

        this.calibrate_sub.subscribe(
            (msg) => {
                console.log(msg.names)
                var index = 0;
                for (var i = 0; i < msg.names.length; ++i) {
                    if (msg.names[i] == this.joint_name) {
                        index = i;
                    }
                }

                this.calibrated = msg.calibrated[index];
            }
        );

        interval = window.setInterval(() => {
            if (!this.calibrated && this.toggleEnabled) {
                this.publishCalibrationMessage();
            }
        }, 200);
    },

    methods: {
        toggleCalibration: function () {
            this.toggleEnabled = !this.toggleEnabled;
        },
        publishCalibrationMessage: function () {
            let request = new ROSLIB.ServiceRequest({
                name: this.name,
                calibrate: this.toggleEnabled
            });
            this.calibrate_service.callService(request, (result) => {
                if (!result) {
                    this.toggleEnabled = !this.toggleEnabled;
                    alert("ESW cannot calibrate this motor");
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
        display: block;
    }
</style>