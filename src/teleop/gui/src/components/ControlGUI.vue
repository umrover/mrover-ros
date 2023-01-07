<template>
<div>
    <DriveControls></DriveControls>
    <ArmControls></ArmControls>
    <JointStateTable v-bind:JointStateData="JointState" v-bind:vertical ="true"></JointStateTable>
    <MoteusStateTable></MoteusStateTable>

</div>
</template>

<script>
import DriveControls from './DriveControls.vue';
import ArmControls from './ArmControls.vue';
import GimbalControls from './GimbalControls.vue';
import JointStateTable from './JointStateTable.vue';
import MoteusStateTable from './MoteusStateTable.vue'
import ROSLIB from "roslib"
export default {
    data() {
        return {
            JointState:{}
        }
    },

    components: {
        DriveControls,
        ArmControls,
        GimbalControls,
        JointStateTable,
        MoteusStateTable
    },
    
    created: function () {
        this.brushless_motors = new ROSLIB.Topic({
            ros: this.$ros,
            name: 'drive_status',
            messageType: 'mrover/MotorsStatus'
        });

        this.brushless_motors.subscribe((msg) => {
            this.JointState = msg.joint_states


        })
    }
}


</script>
