<template>
<div>
    <ArmControls></ArmControls>
    <DriveControls></DriveControls>
    <DriveVelDataH></DriveVelDataH>
    <DriveVelDataV></DriveVelDataV>
    <GimbalControls></GimbalControls>
    <PDBFuse></PDBFuse>

</div>
</template>

<script>
import ArmControls from './ArmControls.vue';
import DriveControls from './DriveControls.vue';
import GimbalControls from './GimbalControls.vue';
import DriveVelDataH from './DriveVelDataH.vue';
import DriveVelDataV from './DriveVelDataV.vue';
import PDBFuse from './PDBFuse.vue';

export default {
    data() {
        return {}
    },

    components: {
        ArmControls,
        DriveControls,
        DriveVelDataV,
        DriveVelDataH,
        GimbalControls,
        PDBFuse,
    }
}
</script>
