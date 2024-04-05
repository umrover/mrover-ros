<template>
    <!-- <div class="wrap"> -->
    <span
      >TX: {{ tx.toFixed(2) }} Mbps RX:
      {{ rx.toFixed(2) }} Mbps</span
    >
  <!-- </div> -->
</template>
  
<script lang="ts">
import { defineComponent } from 'vue'
import { mapState } from 'vuex'
  
export default defineComponent({
    data() {
        return {
            tx: 0,
            rx: 0,
        }
    },

    computed: {
        ...mapState('websocket', ['message'])
    },

    watch: {
        message(msg) {
        if (msg.type == 'network_bandwidth') {
            this.tx = parseFloat(msg.tx);
            this.rx = parseFloat(msg.rx);
        }
        }
    }
})
</script>
<style scoped>
</style>