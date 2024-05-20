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
    </div>
  </div>
</template>

<script lang='ts'>

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
      switch (msg.type) {
        case 'soil_temp':
          this.temp = msg.temperature
          break
        case 'soil_humidity':
          this.humidity = msg.relative_humidity
          break
      }
    }
  }
}
</script>

<style scoped></style>
  