<template>
    <div class="wrap">
        <div>
            <h3> Motor Data </h3>
        </div>
        <table class="tableFormat" style="undefined;table-layout: fixed; width: 250px">
            <colgroup>
                <col style="width: 80px">
                <col style="width: 85px">
                <col style="width: 85px">
            </colgroup>
            <thead>
                <tr class="bold">
                    <td class = "tableElement">Side</td>
                    <td class = "tableElement">Left</td>
                    <td class = "tableElement">Right</td>
                </tr>
            </thead>
            <tbody>
                <tr class="bold">
                    <td class = "tableElement">Front</td>
                    <td class = "tableElement">0</td>
                    <td class = "tableElement">1</td>
                </tr>
                <tr>
                    <td class = "tableElement">Current</td>
                    <td class = "tableElement">{{drive_vel_data.frontLeft.current.toFixed(2)}} amps</td>
                    <td class = "tableElement">{{drive_vel_data.frontRight.current.toFixed(2)}} amps</td>
                </tr>
                <tr>
                    <td class = "tableElement">Velocity</td>
                    <td class = "tableElement">{{drive_vel_data.frontLeft.velocity.toFixed(2)}} m/sec</td>
                    <td class = "tableElement">{{drive_vel_data.frontRight.velocity.toFixed(2)}} m/sec</td>
                </tr>
                <tr class="bold">
                    <td class = "tableElement">Middle</td>
                    <td class = "tableElement">2</td>
                    <td class = "tableElement">3</td>
                </tr>
                <tr>
                    <td class = "tableElement">Current</td>
                    <td class = "tableElement">{{drive_vel_data.middleLeft.current.toFixed(2)}} amps</td>
                    <td class = "tableElement">{{drive_vel_data.middleRight.current.toFixed(2)}} amps</td>
                </tr>
                <tr>
                    <td class = "tableElement">Velocity</td>
                    <td class = "tableElement">{{drive_vel_data.middleLeft.velocity.toFixed(2)}} m/sec</td>
                    <td class = "tableElement">{{drive_vel_data.middleRight.velocity.toFixed(2)}} m/sec</td>
                </tr>
                <tr class="bold">
                    <td class = "tableElement">Back</td>
                    <td class = "tableElement">4</td>
                    <td class = "tableElement">5</td>
                </tr>
                <tr>
                    <td class = "tableElement">Current</td>
                    <td class = "tableElement">{{drive_vel_data.backLeft.current.toFixed(2)}} amps</td>
                    <td class = "tableElement">{{drive_vel_data.backRight.current.toFixed(2)}} amps</td>
                </tr>
                <tr>
                    <td class = "tableElement">Velocity</td>
                    <td class = "tableElement">{{drive_vel_data.backLeft.velocity.toFixed(2)}} m/sec</td>
                    <td class = "tableElement">{{drive_vel_data.backRight.velocity.toFixed(2)}} m/sec</td>
                </tr>
            </tbody>
        </table>
        <div>
            <h3> Drive State </h3>
        </div>
        <table class="tableFormat" style="undefined;table-layout: fixed; width: 255px">
            <colgroup>
                <col style="width: 85px">
                <col style="width: 85px">
                <col style="width: 85px">
            </colgroup>
            <thead>
                <tr class="bold">
                    <td class = "tableElement">Front</td>
                    <td class = "tableElement">Middle</td>
                    <td class = "tableElement">Back</td>
                </tr>
            </thead>
            <tbody>
                <tr>
                    <td class = "tableElement">{{drive_state_data.frontState}}</td>
                    <td class = "tableElement">{{drive_state_data.middleState}}</td>
                    <td class = "tableElement">{{drive_state_data.backState}}</td>
                </tr>
            </tbody>
        </table>
    </div>
</template>

<script>
    export default {
        data() {
            return {
                drive_vel_data: {
                    frontLeft:{current: 0, velocity: 0},
                    frontRight:{current: 0, velocity: 0},
                    middleLeft:{current: 0, velocity: 0},
                    middleRight:{current: 0, velocity: 0},
                    backLeft:{current: 0, velocity: 0},
                    backRight:{current: 0, velocity: 0}
                },
                drive_state_data: {
                    frontState: "Disconnected",
                    middleState: "Disconnected",
                    backState: "Disconnected"
                }
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
    .tableFormat{
        border-collapse: collapse;
        border-spacing: 0;
    }
    .tableFormat td{
        border-color: black;
        border-style: solid;
        border-width: 1px;
        font-size: 13px;
        overflow: hidden;
        padding: 10px 5px;
        word-break: normal
    }
    .tableFormat th{
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
    .tableFormat .tableElement{
        border-color: inherit;
        text-align: center;
        vertical-align: top
    }
</style>
