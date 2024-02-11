<template>
    <div class="wrap box">
        <h3>Soil Data</h3>
        <div class="temp">
            <label for="temperature">Temperature:</label>
            {{ temp }}
        </div>
        <div class="humidity">
            <label for="humidity">Humidity:</label>
            {{ humidity }}
        </div>
    </div>
</template>
  
<script lang="ts">

import { mapState } from 'vuex'

export default {
    data() {
        return {
            temp: 0,
            humidity: 0
        }
    },

    computed: {
        ...mapState('websocket', ['message'])
    },

    watch: {
        message(msg) {
            if (msg.type == 'temp_data') {
                this.temp = msg.temp_data;
            }
            else if (msg.type == 'relative_humidity') {
                this.humidity = msg.humidity_data;
            }
        }
    },
}
</script>
  
<style scoped></style>
  