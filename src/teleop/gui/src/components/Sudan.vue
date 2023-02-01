<template>
<div>
    <h3>Sudan III Drop</h3>
    <button id="sudan-button" :disabled="!isEnabled[index]" v-on:click="moveServo(angles.pushed[index], false)">Start Site {{site}} Test</button>
    <button id="reset-button" v-on:click="moveServo(angles.start[index], true)">Reset Site {{site}} Servo</button>
</div>
</template>

<script>
import ROSLIB from 'roslib/src/RosLib';
import Vue from 'vue'

export default {
    data() {
        return {
            isEnabled: [true, true, true],
            index: 0, //0: site A, 1: site B, 2: site C
            angles: {
                pushed: [],
                start: []
            },
            servoClient: null
        }
    },

    props: {
        site: {
            type: String,
            required: true,
        }
    },

    watch: {
        site: function () {
            if (this.site == 'A') this.index = 0;
            else if (this.site == 'B') this.index = 1;
            else this.index = 2;
        }
    },

    methods: {
        moveServo(angle, enable) {
            let request = new ROSLIB.ServiceRequest({
                id: this.index,
                angle: angle
            });

            this.serviceClient.callService(request, (result) => {
                if (!result.success) alert("Changing servo angle at site " + this.site + " was not successful");
                else Vue.set(this.isEnabled, this.index, enable);
            });
        },
    },

    created: function () {
        this.serviceClient = new ROSLIB.Service({
                ros: this.$ros,
                name: 'change_servo_angles',
                serviceType: 'mrover/ChangeServoAngle'
            });

        //get servo angles from yaml file
        let letter = 'A';
        for (var i = 0; i < 3; i++) {
            let push = new ROSLIB.Param({
                ros: this.$ros,
                name: 'science/syringe_servo_positions/site_' + letter + '/pushed'
            });
            let start = new ROSLIB.Param({
                ros: this.$ros,
                name: 'science/syringe_servo_positions/site_' + letter + '/start'
            });

            push.get((value) => {
                this.angles.pushed.push(value);
            });
            start.get((value) => {
                this.angles.start.push(value);
            });
            letter = String.fromCharCode(letter.charCodeAt(0) + 1);
        }
    }
}
</script>

<style scoped>
</style>
