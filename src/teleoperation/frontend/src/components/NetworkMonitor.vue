<template>
    <p>TX: {{ tx.toFixed(2) }} </p>
    <p>RX: {{ rx.toFixed(2) }} </p>
</template>

<script lang="ts">
import { mapState } from 'vuex'
export default {
    data() {
        return {
            tx: 0,
            rx: 0
        };
    },

    computed: {
    ...mapState('websocket', ['message'])
    },

    watch: {
        message(msg) {
            if(msg.type == "network_monitor") {
                this.tx = msg.tx;
                this.rx = msg.rx;
            }
        }
    }
};
</script>
  
<style scoped>
/*p {
    position: absolute;
    right: 0;
}*/
</style>