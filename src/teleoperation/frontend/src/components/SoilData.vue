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
            readData: false,
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
                }
                else{
                    this.predictedTemp = this.predictTemp(this.temp)
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
                
                this.publishPolyfitMessage(this.tempArray)
            }
            else{
                this.exponents = []
                this.tempArray = []
            }
            this.readData = !this.readData
        }


    },

    methods: {
        ...mapActions('websocket', ['sendMessage']),
        // TODO: add a function that calculates the polyfit

        publishPolyfitMessage: function(temperatures: any){
            this.sendMessage({
                type: "polyfit",
                temperatures: temperatures
            })
        },
        // TODO: add a function that predicts current temp based off polyfit
        predictTemp: function(temperature: any){
            Math.exp(this.exponents[0])*Math.exp(this.exponents * temperature)
        }
    }
}
</script>
  
<style scoped></style>
  