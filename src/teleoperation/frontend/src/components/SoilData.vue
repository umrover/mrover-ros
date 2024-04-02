<template>
    <div class="wrap box">
        <h3>Soil Data</h3>
        <div class="table-responsive">
            <table class="table">
                <thead>
                    <tr class="table-primary">
                        <th scope="col">Temperature</th>
                        <th scope="col">Humidity</th>
                    </tr>
                </thead>
                <tbody>
                    <tr class="bold-border">
                        <td>{{ temp.toFixed(2) }}ºC</td>
                        <td>{{ (humidity * 100).toFixed(2) }}%</td>
                    </tr>
                </tbody>
            </table>
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
  