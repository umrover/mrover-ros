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

            <tr class="Bold">
                <th class="tableElement">Motor</th>
                <th class="tableElement">Positon (m)</th>
                <th class="tableElement">Velocity (m/s)</th>
                <th class="tableElement">Effort (Nm)</th>

            </tr>
        </thead>

        <tbody>

            <tr v-for="motor in motors">
                <th class="tableElement">{{motor.name}}</th>
                <td class="tableElement">{{motor.position}} </td>
                <td class="tableElement">{{motor.velocity}} </td>
                <td class="tableElement">{{motor.effort}} </td>

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

            <tr class="Bold">
                <th class="tableElement">Motor</th>
                <th v-for="motor in motors" class="tableElement">{{motor.name}}</th>
            </tr>
            <tr class="Bold">
                <th class="tableElement">Position (m)</th>
                <td v-for="motor in motors" class="tableElement">{{motor.position}} </td>
            </tr>
            <tr class="Bold">
                <th class="tableElement">Velocity (m/s)</th>
                <td v-for="motor in motors" class="tableElement">{{motor.velocity}} </td>
            </tr>
            <tr class="Bold">
                <th class="tableElement">Effort (Nm)</th>
                <td v-for="motor in motors" class="tableElement">{{motor.effort}} </td>
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
    JointStateData: {
      type: Object,
      required: true
    },

    vertical: {
        type: Boolean,
        required: true
    }
    },
   
    watch:  {
        JointStateData: function(JointStateData){
        const length = JointStateData.velocity.length
        this.motors = []
        for (let i = 0; i < length; i++) {
                this.motors.push({
                name: JointStateData.name[i],
                position: JointStateData.position[i],
                velocity: JointStateData.velocity[i],
                effort: JointStateData.effort[i]
                })
            }
    }
    }
}
</script>



<style scoped>
.wrap {
    display: inline-block;
    align-content: center;
    /* height: 300px; */
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

.bold {
    font-weight: bold;
    border: 2px solid black;
}

.tableFormat .tableElement {
    border-color: inherit;
    text-align: center;
    vertical-align: top
}
</style>

