<template>
<div>
    <DriveControls></DriveControls>
    <ArmControls></ArmControls>
    <GimbalControls></GimbalControls>
    <PDBFuse></PDBFuse>
</div>

</template>

<script>
    import DriveControls from './DriveControls.vue';
    import ArmControls from './ArmControls.vue';
    import GimbalControls from './GimbalControls.vue';
    import PDBFuse from './PDBFuse.vue';

    export default {
        data() {
            return {
            }
        },


        components:{
            DriveControls,
            ArmControls,
            GimbalControls,
            PDBFuse
        }
    }
</script>