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
        <div>
            <p v-if="predictedTemp !== null"> Predicted Temperature: {{ predictedTemp }}</p>
        </div>
        <Checkbox :name="'Read Temp Data'" @toggle="readData = $event"></Checkbox>
    </div>
</template>
  
<script lang="ts">

import { mapState, mapActions } from 'vuex'
import Checkbox from './Checkbox.vue';
import { prependOnceListener } from 'process';

export default {
    components: {
        Checkbox
    },

    data() {
        return {
            temp: 0,
            humidity: 0,
            tempArray: [] as number[],
            timestamps: [] as number[],
            readData: false,
            prevstate: false,
            exponents: [],
            predictedTemp: null
        }
    },

    computed: {
        ...mapState('websocket', ['message'])
    },

    watch: {
        message(msg) {
            if (msg.type == 'temp_data') {
                this.temp = msg.temp_data;
                if(this.readData) {
                    // TODO: add temp to tempArray
                    this.tempArray.push(this.temp)
                    this.timestamps.push(Date.now())
                }
                else{
                    console.log(this.exponents)
                    this.predictedTemp = this.predictTemp(Date.now())
                    console.log(this.predictedTemp)
                }
            }
            else if (msg.type == 'relative_humidity') {
                this.humidity = msg.humidity_data;
            }
            else if (msg.type == 'polyfit') {
                this.exponents = msg.exponents
            }
        },
        readData(){
            if(!this.readData){
                this.publishPolyfitMessage(this.tempArray, this.timestamps)
            }
            else if(this.readData && this.readData != this.prevstate){
                this.exponents = []
                this.tempArray = []
                this.timestamps = []
            }
            this.prevstate = this.readData
        }


    },

    methods: {
        ...mapActions('websocket', ['sendMessage']),
        // TODO: add a function that calculates the polyfit

        publishPolyfitMessage: function(temperatures: any, timestamps: any){
            this.sendMessage({
                type: "polyfit",
                temperatures: temperatures,
                timestamps: timestamps
            })
        },
        // TODO: add a function that predicts current temp based off polyfit
        predictTemp: function(timestamp: any){
            const val = this.exponents[0] * timestamp + this.exponents[1]
            console.log(val)
            return Math.exp(val)
        }
    }
}
</script>
  
<style scoped></style>
  