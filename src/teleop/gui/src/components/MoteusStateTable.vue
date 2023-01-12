<template>
<div class="wrap">
    <div>
        <h3> Moteus States </h3>
    </div>
    <table class="tableFormat" style="undefined;table-layout: fixed; width: 400px">
        <colgroup>
            <col style="width: 20px">
            <col style="width: 30px">
            <col style="width: 30px">

        </colgroup>

        <thead>
            <tr class="Bold">
                <th class="tableElement" style="font-weight:bold">Name</th>
                <th class="tableElement" style="font-weight:bold">{{MoteusStateData.name[FrontLeft]}}</th>
                <th class="tableElement" style="font-weight:bold">{{MoteusStateData.name[FrontRight]}}</th>
            </tr>

            <tr class="Bold">
                <th class="tableElement" style="font-weight:bold">State</th>
                <th class="tableElement">{{MoteusStateData.state[FrontLeft]}}</th>
                <th class="tableElement">{{MoteusStateData.state[FrontRight]}}</th>
            </tr>

            <tr class="Bold">
                <th class="tableElement" style="font-weight:bold">Error</th>
                <th class="tableElement">{{MoteusStateData.error[FrontLeft]}}</th>
                <th class="tableElement">{{MoteusStateData.error[FrontRight]}}</th>
            </tr>

            <tr class="Bold">
                <th class="tableElement" style="font-weight:bold">Name</th>
                <th class="tableElement" style="font-weight:bold">{{MoteusStateData.name[MiddleLeft]}}</th>
                <th class="tableElement" style="font-weight:bold">{{MoteusStateData.name[MiddleRight]}}</th>
            </tr>

            <tr class="Bold">
                <th class="tableElement" style="font-weight:bold">State</th>
                <th class="tableElement">{{MoteusStateData.state[MiddleLeft]}}</th>
                <th class="tableElement">{{MoteusStateData.state[MiddleRight]}}</th>
            </tr>

            <tr class="Bold">
                <th class="tableElement" style="font-weight:bold">Error</th>
                <th class="tableElement">{{MoteusStateData.error[MiddleLeft]}}</th>
                <th class="tableElement">{{MoteusStateData.error[MiddleRight]}}</th>
            </tr>
            <tr class="Bold">
                <th class="tableElement" style="font-weight:bold">Name</th>
                <th class="tableElement" style="font-weight:bold">{{MoteusStateData.name[BackLeft]}}</th>
                <th class="tableElement" style="font-weight:bold">{{MoteusStateData.name[BackRight]}}</th>
            </tr>

            <tr class="Bold">
                <th class="tableElement" style="font-weight:bold">State</th>
                <th class="tableElement">{{MoteusStateData.state[BackLeft]}}</th>
                <th class="tableElement">{{MoteusStateData.state[BackRight]}}</th>
            </tr>

            <tr class="Bold">
                <th class="tableElement" style="font-weight:bold">Error</th>
                <th class="tableElement">{{MoteusStateData.error[BackLeft]}}</th>
                <th class="tableElement">{{MoteusStateData.error[BackRight]}}</th>
            </tr>

        </thead>
    </table>

</div>
</template>

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

<script>
import ROSLIB from "roslib"

export default {
    data() {
        return {
            motors: [],
            FrontLeft: 0,
            FrontRight: 1,
            MiddleLeft: 2,
            MiddleRight: 3,
            BackLeft: 4,
            BackRight: 5,
        }
    },
    props: {
        MoteusStateData: {
            type: Object,
            required: true
        },

        watch: {
            MoteusStateData: function (NewVal) {
                this.motors = []
                console.log(NewVal)
                for (let i = 0; i < 6; i++) {
                    this.motors.push({
                        name: NewVal.name[i],
                        state: NewVal.state[i],
                        error: NewVal.error[i],
                    })
                }
            }
        }

    }
}
</script>
