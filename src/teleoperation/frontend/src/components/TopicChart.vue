<template>
  <div>
    <apexchart type="line" height="350" :options="chartOptions" :series="series"></apexchart>
  </div>
</template>

<script lang="ts">
import { mapState, mapActions, mapMutations, mapGetters } from 'vuex'
export default {
  data() {
    return {
      series: [
        {
          name: 'Value',
          data: [] as number[]
        }
      ],
      chartOptions: {
        chart: {
          height: 350,
          type: 'line',
          zoom: {
            enabled: false
          }
        },
        dataLabels: {
          enabled: false
        },
        stroke: {
          curve: 'straight'
        },
        title: {
          text: 'Value of /reading over time',
          align: 'left'
        },
        grid: {
          row: {
            colors: ['#f3f3f3', 'transparent'], // takes an array which will be repeated on columns
            opacity: 0.5
          }
        }
      }
    }
  },
  computed: {
    ...mapState('websocket', ['message'])
  },
  watch: {
    message: function (msg) {
      if (msg.type == 'reading') {
        this.series[0].data.push(msg.value)
      }
    }
  }
}
</script>

<style scoped></style>
