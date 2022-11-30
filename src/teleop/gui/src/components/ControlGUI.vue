<template>
<div>
    <DriveControls></DriveControls>
    <ArmControls></ArmControls>
    <JointStateTable v-bind:name="'/drive_data'" v-bind:vertical ="false"></JointStateTable>

</div>
</template>

<script>
import DriveControls from './DriveControls.vue';
import ArmControls from './ArmControls.vue';
import GimbalControls from './GimbalControls.vue';
import JointStateTable from './JointStateTable.vue'

export default {
    data() {
        return {}
    },

    components: {
        DriveControls,
        ArmControls,
        GimbalControls,
        JointStateTable
    }
}
</script>
