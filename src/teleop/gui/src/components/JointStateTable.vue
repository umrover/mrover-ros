<template>
<div class="wrap">
    <div v-if="vertical">
        <div>
            <h3> Motor Data </h3>
        </div>
        <table class="tableFormat" style="undefined;table-layout: fixed; width: 745px">
            <colgroup>
                <col style="width: 85px">
                <col style="width: 60px">
                <col style="width: 75px">
                <col style="width: 85px">
                <col style="width: 60px">
                <col style="width: 75px">
                <col style="width: 85px">
                <col style="width: 60px">
                <col style="width: 75px">
                <col style="width: 85px">
            </colgroup>

            <thead>
                <tr>
                    <th class="tableElement">Motor</th>
                    <th class="tableElement">Positon (m)</th>
                    <th class="tableElement">Velocity (m/s)</th>
                    <th class="tableElement">Effort (Nm)</th>
                </tr>
            </thead>

            <tbody>
                <tr v-for="(state, i) in jointStateData">
                    <th class="tableElement">{{state.name[i]}}</th>
                    <td class="tableElement">{{state.position[i]}} </td>
                    <td class="tableElement">{{state.velocity[i]}} </td>
                    <td class="tableElement">{{state.effort[i]}} </td>
                </tr>
            </tbody>
        </table>
    </div>

    <div v-else>
        <div>
            <h3> Motor Data </h3>
        </div>
        <table class="tableFormat" style="undefined;table-layout: fixed; width: 745px">
            <colgroup>
                <col style="width: 85px">
                <col style="width: 60px">
                <col style="width: 75px">
                <col style="width: 85px">
                <col style="width: 60px">
                <col style="width: 75px">
                <col style="width: 85px">
                <col style="width: 60px">
                <col style="width: 75px">
                <col style="width: 85px">
            </colgroup>

            <thead>
                <tr>
                    <th class="tableElement">Motor</th>
                    <th v-for="name in jointStateData.name" class="tableElement">{{name}}</th>
                </tr>
                <tr>
                    <th class="tableElement">Position (m)</th>
                    <td v-for="position in jointStateData.position" class="tableElement">{{position}} </td>
                </tr>
                <tr>
                    <th class="tableElement">Velocity (m/s)</th>
                    <td v-for="velocity in jointStateData.velocity" class="tableElement">{{velocity}} </td>
                </tr>
                <tr>
                    <th class="tableElement">Effort (Nm)</th>
                    <td v-for="effot in jointStateData.effort" class="tableElement">{{effort}} </td>
                </tr>
            </thead>

        </table>

    </div>
</div>
</template>

<script>
import ROSLIB from "roslib"

export default {
    data() {
        return {
            motors: []

        }
    },
    props: {
        jointStateData: {
            type: Object,
            required: true
        },

        vertical: {
            type: Boolean,
            required: true
        }
    }
}
</script>

<style scoped>
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
}

.tableFormat td {
    border-color: black;
    border-style: solid;
    border-width: 1px;
    font-size: 13px;
    overflow: hidden;
    padding: 10px 5px;
    word-break: normal
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

.tableFormat .tableElement {
    border-color: inherit;
    text-align: center;
    vertical-align: top
}
</style>
