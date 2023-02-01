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
            <tr>
                <th class="tableElement tableHeader">Name</th>
                <th class="tableElement tableHeader">{{moteusStateData.name[indices.FrontLeft]}}</th>
                <th class="tableElement tableHeader">{{moteusStateData.name[indices.FrontRight]}}</th>
            </tr>
            <tr>
                <th class="tableElement tableHeader">State</th>
                <th class="tableElement">{{moteusStateData.state[indices.FrontLeft]}}</th>
                <th class="tableElement">{{moteusStateData.state[indices.FrontRight]}}</th>
            </tr>
            <tr>
                <th class="tableElement tableHeader">Error</th>
                <th class="tableElement">{{moteusStateData.error[indices.FrontLeft]}}</th>
                <th class="tableElement">{{moteusStateData.error[indices.FrontRight]}}</th>
            </tr>
            <tr>
                <th class="tableElement tableHeader">Name</th>
                <th class="tableElement tableHeader">{{moteusStateData.name[indices.MiddleLeft]}}</th>
                <th class="tableElement tableHeader">{{moteusStateData.name[indices.MiddleRight]}}</th>
            </tr>
            <tr>
                <th class="tableElement tableHeader">State</th>
                <th class="tableElement">{{moteusStateData.state[indices.MiddleLeft]}}</th>
                <th class="tableElement">{{moteusStateData.state[indices.MiddleRight]}}</th>
            </tr>
            <tr>
                <th class="tableElement tableHeader">Error</th>
                <th class="tableElement">{{moteusStateData.error[indices.MiddleLeft]}}</th>
                <th class="tableElement">{{moteusStateData.error[indices.MiddleRight]}}</th>
            </tr>
            <tr>
                <th class="tableElement tableHeader">Name</th>
                <th class="tableElement tableHeader">{{moteusStateData.name[indices.BackLeft]}}</th>
                <th class="tableElement tableHeader">{{moteusStateData.name[indices.BackRight]}}</th>
            </tr>
            <tr>
                <th class="tableElement tableHeader">State</th>
                <th class="tableElement">{{moteusStateData.state[indices.BackLeft]}}</th>
                <th class="tableElement">{{moteusStateData.state[indices.BackRight]}}</th>
            </tr>
            <tr>
                <th class="tableElement tableHeader">Error</th>
                <th class="tableElement">{{moteusStateData.error[indices.BackLeft]}}</th>
                <th class="tableElement">{{moteusStateData.error[indices.BackRight]}}</th>
            </tr>
        </thead>
    </table>

</div>
</template>

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

.tableElement.tableHeader {
    font-weight: bold;
}
</style>

<script>
import ROSLIB from "roslib"

const state = {
    FrontLeft: 0,
    FrontRight: 1,
    MiddleLeft: 2,
    MiddleRight: 3,
    BackLeft: 4,
    BackRight: 5,
}

export default {
    data() {
        return {
            indices: state
        }
    },

    props: {
        moteusStateData: {
            type: Object,
            required: true
        }
    }
}
</script>
