<template>
<div>
    <ArmControls></ArmControls>
    <GimbalControls></GimbalControls>
    <JointStateTable v-bind:JointStateData="JointState" v-bind:vertical="true"></JointStateTable>
    <MoteusStateTable v-bind:MoteusStateData="MoteusState"></MoteusStateTable>

</div>
</template>

<script>
import ROSLIB from "roslib"
import DriveControls from './DriveControls.vue';
import ArmControls from './ArmControls.vue';
import GimbalControls from './GimbalControls.vue';
import JointStateTable from './JointStateTable.vue';
import MoteusStateTable from './MoteusStateTable.vue'

export default {
    data() {
        return {
            JointState: {},
            MoteusState: {}
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
            this.MoteusState = msg.moteus_states

        })
    }
}
</script>
