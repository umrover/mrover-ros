
<template>
<div class="wrap">
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
    
    <tr class = "Bold" >
        <th class = "tableElement">motor</th>
        <th class = "tableElement">positon</th>
        <th class = "tableElement">velocity</th>
        <th class = "tableElement">effort</th>
        
        
    </tr>
    </thead>
    
    <tbody>
    
        <tr v-for="motor in motors">
        <th class = "tableElement">{{motor.name}}</th>
        <td class = "tableElement">{{motor.position}} m</td>
        <td class = "tableElement">{{motor.velocity}} m/s</td>
        <td class = "tableElement">{{motor.effort}} Nm</td>
        
        
        
    </tr>
    </tbody>
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
    .tableFormat{
    border-collapse:collapse;
    border-spacing:0;
    }
    .tableFormat td{
    border-color:black;
    border-style:solid;
    border-width:1px;
    font-size:13px;
    overflow:hidden;
    padding:10px 5px;
    word-break:normal
    }
    .tableFormat th{
    border-color:black;
    border-style:solid;
    border-width:1px;
    font-size:13px;
    font-weight:normal;
    overflow:hidden;
    padding:10px 5px;
    word-break:normal;
    }
    .bold{
      font-weight: bold;
      border: 2px solid black;
    }
    .tableFormat .tableElement{
    border-color:inherit;
    text-align:center;
    vertical-align:top
    }
</style>





    
    
    <script>
    import ROSLIB from "roslib"

    export default {
      data() {
        return {
            motors:[]
            
        }
      },

      created: function () {
        this.brushless_motors = new ROSLIB.Topic({
        ros : this.$ros,
        name : '/drive_data',
        messageType : 'sensor_msgs/JointState'
    });     
    
    this.brushless_motors.subscribe((msg) => {
        const length = msg.velocity.length
        this.motors = []
        for (let i = 0; i<length; i++)
        {
            this.motors.push({name:msg.name[i], position:msg.position[i], velocity:msg.velocity[i], effort:msg.effort[i]})
        }

  
    })
    
    
}
    
      
     
    }
    </script>
    