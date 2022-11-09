<template>
    <div>
        <h3>Raman</h3>
        <ToggleButton id="uv_carousel" v-bind:currentState="RamanLaserState" labelEnableText="Raman Laser On" labelDisableText="Raman Laser Off" v-on:change="toggleRamanLaser()"/>
    </div>
</template>

<script>
import ToggleButton from './ToggleButton.vue';
import ROSLIB from 'roslib';


export default {
    data() {
        return {
            RamanLaserState: false,
            ramanService: null
        }
    },

    created: function (){
        this.ramanService = new ROSLIB.Service({
                ros : this.$ros,
                name : 'change_arm_laser_state',
                serviceType : 'mrover/ChangeDeviceState'
        });
    },

    methods:{
        toggleRamanLaser: function () {
                this.RamanLaserState = !this.RamanLaserState
                let request = new ROSLIB.ServiceRequest({
                    enable: this.RamanLaserState
                });
                console.log(request)
                this.ramanService.callService(request, (result) => {
                    if (!result) {
                        this.RamanLaserState = !this.RamanLaserState
                        alert("Toggling Raman Laser failed.")
                    }
                });
            },
    },


    components:{
        ToggleButton
    }

}
</script>

<style scoped>

</style>
