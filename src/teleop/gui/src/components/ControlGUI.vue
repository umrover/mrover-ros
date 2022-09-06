<template>
<div>
    <DriveControls></DriveControls>
    <ArmControls></ArmControls>
</div>

</template>

<script>
    import DriveControls from './DriveControls.vue';
    import ArmControls from './ArmControls.vue';

    export default {
        data() {
            return {
            }
        },


        components:{
            DriveControls,
            ArmControls
        }
    }
</script>