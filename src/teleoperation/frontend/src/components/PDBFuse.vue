<template>
  <div class="wrap">
    <div>
      <h3>Power Distribution Board</h3>
    </div>
    <table class="table">
      <thead>
        <tr class="table-primary">
          <th></th>
          <th>Temperature</th>
          <th>Current</th>
        </tr>
      </thead>
      <tbody>
        <template v-for="(item, i) in pdb_data" :key="item">
          <tr>
            <th class="table-secondary">{{ voltage[i] }}</th>
            <td :class="item.temp.color">{{ item.temp.val.toFixed(2) }}°C</td>
            <td :class="item.current.color">{{ item.current.val.toFixed(2) }} A</td>
          </tr>
        </template>
      </tbody>
    </table>
  </div>
</template>

<script lang="ts">
import { mapState } from 'vuex'
const pdb_temp_limit = 100
const pdb_current_limits = [
  //TBD
  2, 8, 8
]
export default {
  data() {
    return {
      voltage: ['3.3V', '5V', '12V Buck #1', '12V Buck #2', '12V Buck #3'],
      pdb_data: [
        {
          temp: { val: 0.0, color: 'body-bg' },
          current: { val: 0.0, color: 'body-bg' }
        },
        {
          temp: { val: 0.0, color: 'body-bg' },
          current: { val: 0.0, color: 'body-bg' }
        },
        {
          temp: { val: 0.0, color: 'body-bg' },
          current: { val: 0.0, color: 'body-bg' }
        },
        {
          temp: { val: 0.0, color: 'body-bg' },
          current: { val: 0.0, color: 'body-bg' }
        },
        {
          temp: { val: 0.0, color: 'body-bg' },
          current: { val: 0.0, color: 'body-bg' }
        }
      ]
    }
  },
  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == 'pdb') {
        for (let i = this.pdb_data.length - 1; i >= 0; i--) {
          this.updateVal(this.pdb_data[i].temp, msg.temperatures[i], pdb_temp_limit)
          this.updateVal(this.pdb_data[i].current, msg.currents[i], pdb_current_limits[i])
        }
      }
    }
  },

  methods: {
    updateVal: function (struct: { val: any; color: string }, val: number, threshold: number) {
      struct.val = val
      if (val > threshold || val < 0.0) {
        struct.color = 'table-danger'
      } else {
        struct.color = 'body-bg'
      }
    }
  }
}
</script>

<style scoped>
.wrap {
  display: inline-block;
  align-content: center;
}
</style>
