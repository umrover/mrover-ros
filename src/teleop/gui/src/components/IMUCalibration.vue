<template>
<div class = "wrap">
    <div class="table">
        <table class="tableFormat" style="undefined;table-layout:fixed;border=0">
            <caption>IMU Calibration</caption>
            <colgroup>
            <col style="width:80px">
            <col style="width:80px">
            <col style="width:80px">
            <col style="width:80px">
        </colgroup>
        <thead>
            <tr>
                <td class="tableElement">Mag</td>
                <td class="tableElement">Accel</td>
                <td class="tableElement">Gyro</td>
                <td class="tableElement">System</td>
            </tr>
        </thead>
        <tbody>
            <tr>
                <!--<td><span class='led' ,class="{{IMUCalibration[0].color}}"></span></td>-->
                <td><span v-bind:class='["led",magnetommeter_calibration_color?"green":"red"]'></span></td>
                <td><span v-bind:class='["led",accelerometer_calibration_color?"green":"red"]'></span></td>
                <td><span v-bind:class='["led",gyroscope_calibration_color?"green":"red"]'></span></td>
                <td><span v-bind:class='["led",system_calibration_color?"green":"red"]'></span></td>
            </tr>
        </tbody>
        <tbody>
            <tr>
                <td class="tableElement">{{magnetometer_val}}</td>
                <td class="tableElement">{{accelerometer_val}}</td>
                <td class="tableElement">{{ gyroscope_val }}</td>
                <td class="tableElement">{{system_val }}</td>
            </tr>
        </tbody>
        </table>
    </div>
</div>
</template>

<script>
import ROSLIB from "roslib"
const calibration_limit=3

export default {
    data() {
        return {
            system_calibration_color : false,
            gyroscope_calibration_color : false,
            accelerometer_calibration_color : false,
            magnetometer_calibration_color : false,
            system_val: 0,
            gyroscope_val: 0,
            accelerometer_val: 0 ,
            magnetometer_val: 0
        }
    },
    created: function(){
        this.IMUCalibration_sub = new ROSLIB.Topic({
            ros: this.$ros,
            name: 'imu/calibration',
            messageType: 'mrover/CalibrationStatus'
        });
        this.IMUCalibration_sub.subscribe((msg)=> {
            console.log(msg)
            this.system_calibration_color= (calibration_limit == msg.system_calibration)
            this.system_val=msg.system_calibration
            this.gyroscope_calibration_color= (calibration_limit == msg.gyroscope_calibration)
            this.gyroscope_val=msg.gyroscope_calibration
            this.accelerator_calibration_color= (calibration_limit == msg.accelerometer_calibration)
            this.accelerometer_val=msg.accelerometer_calibration
            this.magnetometer_calibration_color= (calibration_limit == msg.magnetometer_calibration)
            this.magnetometer_val=msg.magnetometer_calibration
        })
    },
    
/*
   methods: {
    update_val: function(struct, val){
        struct.val = val;
        if(val==calibration_limit){
            return 'green'
        }
        else{
            return 'red'
        }
    }
   }
   */
}
</script>
<style scoped>
.wrap {
    display: flex;
    align-items: center;
    height: 100%;
  }
  .led {
    width: 16px;
    height: 16px;
    border-radius: 8px;
    border: 1px solid;
    display: block;
  }
.green{
    background-color:lightgreen;
}
.red{
    background-color:red;
}
</style>