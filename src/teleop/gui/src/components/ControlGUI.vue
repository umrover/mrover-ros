<template>
<div>
    <DriveControls></DriveControls>
    <ArmControls></ArmControls>
    <GimbalControls></GimbalControls>
    <JointStateTable v-bind:jointStateData="jointState" v-bind:vertical="true"></JointStateTable>
    <MoteusStateTable v-bind:moteusStateData="moteusState"></MoteusStateTable>
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
            jointState: {},
            moteusState: {}
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
            this.jointState = msg.joint_states
            this.moteusState = msg.moteus_states
        })
    }
}
</script>
