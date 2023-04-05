<template>
<div>
    <Attitude :size="300" :pitch="pitch" :roll="roll"></Attitude>
</div>
</template>

<script>
import {
    Attitude
} from 'vue-flight-indicators'

export default {
    components: {
        Attitude,
    },
    data() {
        return {
            pitch: 20,
            roll: 0
        };
    },

};
</script>

<style scoped>
</style>
