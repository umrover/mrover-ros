<template>
<div>
    <DriveControls></DriveControls>
    <ArmControls></ArmControls>
    <GimbalControls></GimbalControls>
    <DriveVelDataH></DriveVelDataH>
    <DriveVelDataV></DriveVelDataV>

</div>
</template>

<script>
import DriveControls from './DriveControls.vue';
import ArmControls from './ArmControls.vue';
import GimbalControls from './GimbalControls.vue';
import DriveVelDataH from './DriveVelDataH.vue'
import DriveVelDataV from './DriveVelDataV.vue'

export default {
    data() {
        return {}
    },

    components: {
        DriveControls,
        ArmControls,
        GimbalControls,
        DriveVelDataV,
        DriveVelDataH
    }
}
</script>
