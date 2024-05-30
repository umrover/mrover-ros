<template>
  <div class='wrap box'>
    <h3>Soil Data</h3>
    <div class='table-responsive'>
      <table class='table'>
        <thead>
        <tr class='table-primary'>
          <th scope='col'>Temperature</th>
          <th scope='col'>Humidity</th>
        </tr>
        </thead>
        <tbody>
        <tr class='bold-border'>
          <td>{{ temp.toFixed(2) }}ºC</td>
          <td>{{ (humidity * 100).toFixed(2) }}%</td>
        </tr>
        </tbody>
      </table>
      <div>
        <p v-if="predictedTemp"> Predicted Temperature: {{ predictedTemp }}</p>
        <Checkbox :name="'Read Temp Data'" @toggle="readData = $event"></Checkbox>
      </div>
    </div>
  </div>
</template>

<script lang='ts'>

import { mapState, mapActions } from 'vuex'
import Checkbox from './Checkbox.vue'

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
      prevState: false,
      exponents: [],
      predictedTemp: null
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      switch (msg.type) {
        case 'soil_temp':
          this.temp = msg.temperature
          if(this.readData){
            this.tempArray.push(this.temp)
            this.timestamps.push(Date.now())
          }
          else {
            this.predictedTemp = this.predictTemp(Date.now())
          }
          break
        case 'soil_humidity':
          this.humidity = msg.relative_humidity
          break
        case 'poly_fit':
          this.exponents = msg.exponents
          break
      }
    },

    readData() {
      if(!this.readData){
        this.publishPolyfit()
      }
      else if(this.readData && this.readData != this.prevState) {
        this.exponents = []
        this.tempArray = []
        this.timestamps = []
      }
      this.prevState = this.readData
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    publishPolyfit: function(){
      this.sendMessage({type: "poly_fit", temperatures: this.tempArray, timestamps: this.timestamps})
    },

    predictTemp: function(timestamp: any) {
      const val = this.exponents[0] * timestamp + this.exponents[1]
      return Math.exp(val)
    }
  }
}
</script>

<style scoped></style>
  