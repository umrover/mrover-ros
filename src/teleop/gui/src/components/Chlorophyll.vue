<template>
    <div>
        <h3>Chlorophyll Test Control</h3>
        <div>
            <div>
                <ToggleButton id="white_led_toggle" v-bind:currentState="whiteLEDS_active" labelEnableText="White LEDs On" labelDisableText="White LEDs Off" v-on:change="toggle_whiteLEDS()"/>
                <ToggleButton id="uv_end_effector" v-bind:currentState="UV_endEffector" labelEnableText="End Effector UV On" labelDisableText="End Effector UV On" v-on:change="toggleEndEffectorUV()"/>
                <ToggleButton id="uv_carousel" v-bind:currentState="UV_carousel" labelEnableText="Carousel UV On" labelDisableText="Carousel UV Off" v-on:change="toggleCarouselUV()"/>
            </div>
        </div>
        <div class="wrap-table">
            <div>
                <h3> Chlorophyll Spectral data </h3>
            </div>
            <table class="tableFormat" style="undefined;table-layout: fixed; width: 434px">
                <colgroup>
                    <col style="width: 63px">
                    <col style="width: 53px">
                    <col style="width: 53px">
                    <col style="width: 53px">
                    <col style="width: 53px">
                    <col style="width: 53px">
                    <col style="width: 53px">
                    <col style="width: 53px">
                </colgroup>
                <thead>
                <tr>
                    <th class = "tableElement"></th>
                    <th class = "tableElement">1</th>
                    <th class = "tableElement">2</th>
                    <th class = "tableElement">3</th>
                    <th class = "tableElement">4</th>
                    <th class = "tableElement">5</th>
                    <th class = "tableElement">6</th>
                </tr>
                </thead>
                <tbody>
                    <tr>
                        <td class = "tableElement">Spec 0</td>
                        <td class = "tableElement">{{ spectral_data.d0_1.toFixed(0) }}</td>
                        <td class = "tableElement">{{ spectral_data.d0_2.toFixed(0) }}</td>
                        <td class = "tableElement">{{ spectral_data.d0_3.toFixed(0) }}</td>
                        <td class = "tableElement">{{ spectral_data.d0_4.toFixed(0) }}</td>
                        <td class = "tableElement">{{ spectral_data.d0_5.toFixed(0) }}</td>
                        <td class = "tableElement">{{ spectral_data.d0_6.toFixed(0) }}</td>
                    </tr>
                </tbody>
            </table>
        </div>
        <GenerateReport v-bind:spectral_data="spectral_data"/>
    </div>
</template>

<script>
import ToggleButton from './ToggleButton.vue';
import GenerateReport from './GenerateReport.vue';
import ROSLIB from 'roslib/src/RosLib';

export default {
    data() {
        return {
            whiteLEDS_active: false,
            UV_endEffector: false,
            UV_carousel: false
        }
    },

    props: {
        spectral_data: {
            type: Object,
            required: true
        }
    },

    components: {
        ToggleButton,
        GenerateReport
    },

    methods: {
        toggle_whiteLEDS: function () {
            this.whiteLEDS_active = !this.whiteLEDS_active
            let whiteLedService = new ROSLIB.Service({
                ros : this.$ros,
                name : 'change_white_led_state',
                serviceType : 'mrover/ChangeDeviceState'
            });
            let request = new ROSLIB.ServiceRequest({
                enable: this.whiteLEDS_active
            });
            whiteLedService.callService(request, (result) => {
                if (!result) {
                    this.whiteLEDS_active = !this.whiteLEDS_active
                    alert("Toggling white LEDs failed.")
                }
            });
        },

        toggleEndEffectorUV: function () {
            this.UV_endEffector = !this.UV_endEffector
            let uvService = new ROSLIB.Service({
                ros : this.$ros,
                name : 'change_uv_led_end_effector_state',
                serviceType : 'mrover/ChangeDeviceState'
            });
            let request = new ROSLIB.ServiceRequest({
                enable: this.UV_active
            });
            uvService.callService(request, (result) => {
                if (!result) {
                    this.UV_endEffector = !this.UV_endEffector
                    alert("Toggling End Effector UV failed.")
                }
            });
        },

        toggleCarouselUV: function () {
            this.UV_carousel = !this.UV_carousel
            let uvService = new ROSLIB.Service({
                ros : this.$ros,
                name : 'change_uv_led_carousel_state',
                serviceType : 'mrover/ChangeDeviceState'
            });
            let request = new ROSLIB.ServiceRequest({
                enable: this.UV_active
            });
            uvService.callService(request, (result) => {
                if (!result) {
                    this.UV_carousel = !this.UV_carousel
                    alert("Toggling Carousel UV failed.")
                }
            });
        }
    }
}
</script>

<style scoped>
  .box1 {
    text-align: left;
    vertical-align: top;
    display: inline-block;
  }

  .wrap-table {
    display: inline-block;
    align-content: center;
    height: max-content;
    padding-bottom: 5px;
  }

  .report {
    height: 5vh;
    align-items: center;
    padding-top: 3vh;
  }

  .tableFormat{
    border-collapse:collapse;
    border-spacing:0;
  }

  .tableFormat td{
    border-color:black;
    border-style:solid;
    border-width:1px;
    font-size:14px;
    overflow:hidden;
    padding:5px 5px;
    word-break:normal
  }

  .tableFormat th{
    border-color:black;
    border-style:solid;
    border-width:1px;
    font-size:14px;
    font-weight:normal;
    overflow:hidden;
    padding:10px 5px;
    word-break:normal;
  }

  .tableFormat .tableElement{
    border-color:inherit;
    text-align:center;
    vertical-align:top
  }
</style>
