<template>
    <div class="wrap">
        <div>
            <h3>Power Distribution Board</h3>
        </div>
        <div class="table">
            <table class="tableFormat" style="undefined;table-layout: fixed">
                <colgroup>
                    <col style="width: 80px">
                    <col style="width: 100px">
                    <col style="width: 100px">
                    <col style="width: 100px">
                </colgroup>
                <thead>
                    <tr>
                        <td class = "tableElement"></td>
                        <td class = "tableElement">Temperature</td>
                        <td class = "tableElement">Current</td>
                        <td class = "tableElement">Voltage</td>
                    </tr>
                </thead>
                    <tbody>
                    <tr>
                        <td class = "tableElement">3.3V</td>
                        <td class = "tableElement" v-bind:style="{color: pdb_data[0].temp.color}">{{pdb_data[0].temp.val.toFixed(5)}}°C</td>
                        <td class = "tableElement" v-bind:style="{color: pdb_data[0].current.color}">{{pdb_data[0].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: pdb_data[0].voltage.color}">{{pdb_data[0].voltage.val.toFixed(5)}}V</td>
                    </tr>
                    <tr>
                        <td class = "tableElement">5V</td>
                        <td class = "tableElement" v-bind:style="{color: pdb_data[1].temp.color}">{{pdb_data[1].temp.val.toFixed(5)}}°C</td>
                        <td class = "tableElement" v-bind:style="{color: pdb_data[1].current.color}">{{pdb_data[1].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: pdb_data[1].voltage.color}">{{pdb_data[1].voltage.val.toFixed(5)}}V</td>
                    </tr>
                    <tr>
                        <td class = "tableElement">12V</td>
                        <td class = "tableElement" v-bind:style="{color: pdb_data[2].temp.color}">{{pdb_data[2].temp.val.toFixed(5)}}°C</td>
                        <td class = "tableElement" v-bind:style="{color: pdb_data[2].current.color}">{{pdb_data[2].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: pdb_data[2].voltage.color}">{{pdb_data[2].voltage.val.toFixed(5)}}V</td>
                    </tr>
                </tbody>
            </table>
        </div>
        <div>
            <h3>Fuse Board</h3>
        </div>
        <div class="table">
            <table class="tableFormat" style="undefined;table-layout: fixed">
                <colgroup>
                    <col style="width: 80px">
                    <col style="width: 100px">
                    <col style="width: 100px">
                </colgroup>
                <thead>
                    <tr>
                        <td class="tableElement"></td>
                        <td class="tableElement">Current</td>
                        <td class="tableElement">Voltage</td>
                    </tr>
                </thead>
                <tbody>
                    <tr>
                        <td class="tableElement">Joint A</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[0].current.color}">{{fuse_data[0].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[0].voltage.color}">{{fuse_data[0].voltage.val.toFixed(5)}}V</td>
                    </tr>
                    <tr>
                        <td class="tableElement">Joint B</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[1].current.color}">{{fuse_data[1].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[1].voltage.color}">{{fuse_data[1].voltage.val.toFixed(5)}}V</td>
                    </tr>
                    <tr>
                        <td class="tableElement">Joint C</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[2].current.color}">{{fuse_data[2].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[2].voltage.color}">{{fuse_data[2].voltage.val.toFixed(5)}}V</td>
                    </tr>
                    <tr>
                        <td class="tableElement">Joint D</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[3].current.color}">{{fuse_data[3].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[3].voltage.color}">{{fuse_data[3].voltage.val.toFixed(5)}}V</td>
                    </tr>
                    <tr>
                        <td class="tableElement">Joint E</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[4].current.color}">{{fuse_data[4].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[4].voltage.color}">{{fuse_data[4].voltage.val.toFixed(5)}}V</td>
                    </tr>
                    <tr>
                        <td class="tableElement">Joint F</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[5].current.color}">{{fuse_data[5].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[5].voltage.color}">{{fuse_data[5].voltage.val.toFixed(5)}}V</td>
                    </tr>
                    <tr>
                        <td class="tableElement">Heater A</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[8].current.color}">{{fuse_data[8].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[8].voltage.color}">{{fuse_data[8].voltage.val.toFixed(5)}}V</td>
                    </tr>
                    <tr>
                        <td class="tableElement">Heater B</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[7].current.color}">{{fuse_data[7].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[7].voltage.color}">{{fuse_data[7].voltage.val.toFixed(5)}}V</td>
                    </tr>
                    <tr>
                        <td class="tableElement">Heater C</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[6].current.color}">{{fuse_data[6].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[6].voltage.color}">{{fuse_data[6].voltage.val.toFixed(5)}}V</td>
                    </tr>
                    <tr>
                        <td class="tableElement">Gripper/scoop + gimbal pitch</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[9].current.color}">{{fuse_data[9].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[9].voltage.color}">{{fuse_data[9].voltage.val.toFixed(5)}}V</td>
                    </tr>
                    <tr>
                        <td class="tableElement">Finger/sensor + gimbal yaw</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[10].current.color}">{{fuse_data[10].current.val.toFixed(5)}}A</td>
                        <td class = "tableElement" v-bind:style="{color: fuse_data[10].voltage.color}">{{fuse_data[10].voltage.val.toFixed(5)}}V</td>
                    </tr>
                </tbody>
            </table>
        </div>
    </div>
</template>

<script>
    const pdb_temp_limit = 100
    const pdb_current_limits = [
        2,
        8,
        8
    ]
    const pdb_volt_limits = [
        3.6,
        5.3,
        14
    ]
    const fuse_current_limit = 2.5
    const fuse_volt_limit = 45

    export default {
        data() {
            return {
                pdb_data: [
                    {
                        temp: {val: 0.0, color: "grey"},
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    },
                    {
                        temp: {val: 0.0, color: "grey"},
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    },
                    {
                        temp: {val: 0.0, color: "grey"},
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    }
                ],

                fuse_data: [
                    {
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    },
                    {
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    },
                    {
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    },
                    {
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    },
                    {
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    },
                    {
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    },
                    {
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    },
                    {
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    },
                    {
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    },
                    {
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    },
                    {
                        current: {val: 0.0, color: "grey"},
                        voltage: {val: 0.0, color: "grey"}
                    }
                ],
            }
        }
    }
</script>

<style scoped>
    .tables {
        display: flex;
    }
    .wrap {
        display: inline-block;
        align-content: center;
    }

    .box {
        border-radius: 5px;
        padding: 10px;
        border: 1px solid black;
        text-align: right;
        vertical-align: top;
    }

    .tableFormat {
        border-collapse: collapse;
        border-spacing: 0;
        padding: 10px;
    }

    .tableFormat td {
        border-color: black;
        border-style: solid;
        border-width: 1px;
        font-size: 13px;
        overflow: hidden;
        padding: 10px 5px;
        word-break: normal;
    }

    .tableFormat th {
        border-color: black;
        border-style: solid;
        border-width: 1px;
        font-size: 13px;
        font-weight: normal;
        overflow: hidden;
        padding: 10px 5px;
        word-break: normal;
    }

    .bold{
        font-weight: bold;
        border: 2px solid black;
    }
    
    .tableFormat .tableElement {
        border-color: inherit;
        text-align: center;
        vertical-align: top
    }
</style>
