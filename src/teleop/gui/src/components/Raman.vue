<template>
<div>
    <h3>Raman</h3>
    <ToggleButton v-bind:currentState="ramanLaserState" labelEnableText="Raman Laser On" labelDisableText="Raman Laser Off" v-on:change="toggleRamanLaser()" />
</div>
</template>

<script>
import ToggleButton from './ToggleButton.vue';
import ROSLIB from 'roslib';

export default {
    data() {
        return {
            ramanLaserState: false,
            ramanService: null
        }
    },

    created: function () {
        this.ramanService = new ROSLIB.Service({
            ros: this.$ros,
            name: 'change_arm_laser_state',
            serviceType: 'mrover/ChangeDeviceState'
        });
    },

    methods: {
        toggleRamanLaser: function () {
            this.ramanLaserState = !this.ramanLaserState
            let request = new ROSLIB.ServiceRequest({
                enable: this.ramanLaserState
            });
            console.log(request)
            this.ramanService.callService(request, (result) => {
                if (!result) {
                    this.ramanLaserState = !this.ramanLaserState
                    alert("Toggling Raman Laser failed.")
                }
            });
        },
    },

    components: {
        ToggleButton
    }

}
</script>

<style scoped>

</style>
