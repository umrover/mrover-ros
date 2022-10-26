<template>
<div>
    <DriveControls></DriveControls>
    <ArmControls></ArmControls>
    <GimbalControls></GimbalControls>>
</div>

</template>

<script>
    import DriveControls from './DriveControls.vue';
    import ArmControls from './ArmControls.vue';
    import GimbalControls from './GimbalControls.vue';

    export default {
        data() {
            return {
            }
        },


        components:{
            DriveControls,
            ArmControls,
            GimbalControls
        }
    }
</script>