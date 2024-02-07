<template>
  <div>
    <div class="shadow p-3 mb-5 header">
      <h1>Topic Viewer</h1>
      <img class="logo" src="/mrover.png" alt="MRover" title="MRover" width="200" />
      <div class="help">
        <img src="/help.png" alt="Help" title="Help" width="48" height="48" />
      </div>
      <div class="helpscreen"></div>
      <div
        class="helpimages"
        style="display: flex; align-items: center; justify-content: space-evenly"
      >
        <img
          src="/joystick.png"
          alt="Joystick"
          title="Joystick Controls"
          style="width: auto; height: 70%; display: inline-block"
        />
      </div>
    </div>
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
        },
        xaxis: {
          tickAmount: 'dataPoints'
        },
        legend: {
          show: true,
          showForSingleSeries: true
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
