<template>
    <div>
        <h3>Chlorophyll Test Control</h3>
        <div>
            <div>
                <ToggleButton id="white_led_toggle" v-bind:currentState="whiteLEDS_active" labelEnableText="White LEDs On" labelDisableText="White LEDs Off" v-on:change="toggle_whiteLEDS()"/>
                <ToggleButton id="uv_toggle" v-bind:currentState="UV_active" labelEnableText="UV LEDs On" labelDisableText="UV LEDs Off" v-on:change="toggle_UV_LEDs()"/>
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

export default {
    data() {
        return {
            whiteLEDS_active: false,
            UV_active: false
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
        },

        toggle_UV_LEDs: function () {
            this.UV_active = !this.UV_active
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
