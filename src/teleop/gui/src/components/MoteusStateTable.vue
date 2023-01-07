<template>
    <div class="wrap">
        <div>
            <h3> Moteus States </h3>
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
                    <th class="tableElement">Name</th>
                    <th class="tableElement">State</th>
                    <th class="tableElement">Error</th>
                </tr>
            </thead>
    
            <tbody>
    
                <tr v-for="motor in motors">
                    <th class="tableElement">{{motor.name}}</th>
                    <td class="tableElement">{{motor.state}} </td>
                    <td class="tableElement">{{motor.error}} </td>
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
            motors: []
             
        }
    },
    
    created: function () {
        this.brushless_motors = new ROSLIB.Topic({
            ros: this.$ros,
            name: '/drive_status',
            messageType: 'mrover/MotorsStatus'
        });
        
        this.brushless_motors.subscribe((msg) => {
            //const length = Moteus_State.name.length
            this.motors = []
            let Moteus_State = msg.moteus_states
            for (let i = 0; i <6; i++) {
                this.motors.push({
                    name: Moteus_State.name[i],
                    state: Moteus_State.state[i],
                    error: Moteus_State.error[i],
                    
            })
            
            }
            
        })
        
    }
 
    }

</script>
    