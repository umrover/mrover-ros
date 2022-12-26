<template>
<div class="wrap">
    <h3>Amino Test Controls</h3>
    <div class="box1 heaters">
        <ToggleButton id="heater" v-bind:currentState="heaters[siteIndex].intended" :labelEnableText="'Heater '+(site)+' Intended'" :labelDisableText="'Heater '+(site)+' Intended'" v-on:change="toggleHeater(siteIndex)" />
        <p v-bind:style="{color: heaters[siteIndex].color}">Thermistor {{site}}: {{heaters[siteIndex].temp.toFixed(2)}} C°</p>
    </div>
    <div class="box1 shutdown">
        <ToggleButton id="autoshutdown" v-bind:currentState="autoShutdownIntended" :labelEnableText="'Auto Shutdown Intended'" :labelDisableText="'Auto Shutdown Intended'" v-on:change="sendAutoShutdownCmd(!autoShutdownIntended)" />
    </div>
</div>
</template>

<script>
import ToggleButton from './ToggleButton.vue'
import ROSLIB from 'roslib'

export default {
    data() {
        return {
            siteIndex: 0,
            heaters: [{
                    enabled: false,
                    intended: false,
                    temp: 0,
                    color: 'grey'
                },
                {
                    enabled: false,
                    intended: false,
                    temp: 0,
                    color: 'grey'
                },
                {
                    enabled: false,
                    intended: false,
                    temp: 0,
                    color: 'grey'
                }
            ],

            autoShutdownEnabled: true,
            autoShutdownIntended: true,

            temp_sub: null
        }
    },

    created: function () {
        this.temp_sub = new ROSLIB.Topic({
            ros: this.$ros,
            name: 'science_data/temperatures',
            messageType: 'mrover/ScienceTemperatures'
        });

        this.temp_sub.subscribe((msg) => {
            // Callback for temp_sub
            console.log(msg)
            for (let i = 0; i < 3; i++) {
                this.heaters[i].temp = msg.temperatures[i]
            }
        });
    },

    props: {
        site: {
            type: String,
            required: true,
        }
    },

    components: {
        ToggleButton,
    },

    methods: {
        toggleHeater: function (id) {
            this.heaters[id].intended = !this.heaters[id].intended
            let toggleHeaterServ = new ROSLIB.Service({
                ros: this.$ros,
                name: 'change_heater_state',
                serviceType: 'mrover/ChangeHeaterState'
            });
            let request = new ROSLIB.ServiceRequest({
                device: id,
                enable: this.heaters[id].intended
            });
            toggleHeaterServ.callService(request, (result) => {
                if (!result) {
                    alert(`Toggling heater ${id} failed.`)
                }
            });
        },

        sendAutoShutdownCmd: function (enabled) {
            this.autoShutdownIntended = enabled
            let autoShutdownServ = new ROSLIB.Service({
                ros: this.$ros,
                name: 'change_heater_auto_shutoff_state',
                serviceType: 'mrover/ChangeDeviceState'
            });
            let request = new ROSLIB.ServiceRequest({
                enable: this.autoShutdownIntended
            });
            autoShutdownServ.callService(request, (result) => {
                if (!result) {
                    alert(`Toggling autoshutdown failed.`)
                }
            });
        }
    },

    watch: {
        site(newVal) {
            if (newVal === "A") {
                this.siteIndex = 0
            } else if (newVal === "B") {
                this.siteIndex = 1
            } else {
                this.siteIndex = 2
            }
        }
    }
}
</script>

<style scoped>
.wrap {
    display: grid;
    grid-gap: 5px;
    grid-template-columns: auto auto;
    grid-template-rows: auto auto auto auto;
    grid-template-areas: "title ."
        "select select"
        "heaters heaterStatus"
        "shutdown shutdownStatus";
    height: auto;
}

.select {
    grid-area: select;
}

.heaters {
    grid-area: heaters;
}

.heaterStatus {
    grid-area: heaterStatus;
}

.shutdown {
    grid-area: shutdown;
}

.shutdownStatus {
    grid-area: shutdownStatus;
}

.box1 {
    text-align: left;
    vertical-align: top;
    display: inline-block;
}

.comms {
    display: flex;
    flex-direction: column;
    align-items: flex-start;
}

.comms * {
    margin-top: 2px;
    margin-bottom: 2px;
}
</style>
