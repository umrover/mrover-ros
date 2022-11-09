<template>
    <div>
        <h3>Sudan III Drop</h3>
        <button id="sudan-button" :disabled="isEnabled[index]" v-on:click="clickButton()">Start Site {{site}} Test</button>
    </div>
</template>

<script>
import ROSLIB from 'roslib/src/RosLib';
import Vue from 'vue'

export default {
    data() {
        return {
            isEnabled: [false, false, false], 
            index: 0, //0: site A, 1: site B, 2: site C
            angles: []
        }
    },

    props: {
        site: {
            type: String,
            required: true,
        }
    },

    watch: {
        site: function(){
            if(this.site == 'A') this.index = 0;
            else if(this.site == 'B') this.index = 1;
            else this.index = 2;
        }
    },

    methods: {
        clickButton(){
            let serviceClient = new ROSLIB.Service({
                ros: this.$ros,
                name: 'change_servo_angles',
                serviceType: 'ChangeServoAngle'
            });

            let request = new ROSLIB.ServiceRequest({id: this.site, angle: this.angles[this.index]});

            serviceClient.callService(request, (result) => {
                if(!result.success) alert("Changing servo angle at site " + this.site + " was not successful");
                else Vue.set(this.isEnabled, this.index, !this.isEnabled[this.index]);
            });
        }
    },

    created: function(){
        //get servo angles from yaml file
        let letter = 'A';
        for(var i = 0; i < 3; i++){
            let a = new ROSLIB.Param({
                ros: this.$ros,
                name: '/syringe_servos/site_' + letter
            });

            a.get((value) => {
                this.angles.push(value);
            });
            letter = String.fromCharCode(letter.charCodeAt(0) + 1);
        }
    }
}
</script>

<style scoped>
</style>
