<template>
  <div class="wrap">
    <div v-if="motors.length > 1">
      <h4>Adjust Motor Angles</h4>
    </div>
    <div v-else>
      <h4>Adjust {{ motors[0].option }} Angle</h4>
    </div>
    <div>
      <div v-if="motors.length > 1">
        <label for="joint">Motor to adjust</label>
        <select v-model="selectedMotor">
          <option disabled value="">Select a motor</option>
          <option v-for="option in motors" :key="option.esw_name" :value="option.esw_name">
            {{ option.display_name }}
          </option>
        </select>
      </div>
      <div>
        <label for="angle">Angle (in Rad)</label>
        <input v-model="adjustmentAngle" type="number" :min="-2 * Math.PI" :max="2 * Math.PI" />
        <button class="btn btn-primary mx-3" type="button" @click="publishAdjustmentMessage">
          Adjust
        </button>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import { mapState, mapActions } from 'vuex'

export default defineComponent({
  props: {
    motors: {
      type: Array<{ esw_name: string; display_name: string }>,
      required: true,
    }
  },

  data() {
    return {
      adjustmentAngle: 0,
      selectedMotor: ''
    }
  },

  created: function () {
    if (this.motors.length == 1) {
      this.selectedMotor = this.motors[0].esw_name
    }
  },

  watch: {
    message(msg) {
      if (msg.type == 'arm_adjust') {
        if (!msg.success) {
          alert('Adjustment failed')
        }
      }
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
    publishAdjustmentMessage() {
      if (this.selectedMotor != '') {
        this.sendMessage({
          type: 'arm_adjust',
          name: this.selectedMotor,
          value: this.clamp(parseFloat(this.adjustmentAngle.toString()), -2 * Math.PI, 2 * Math.PI)
        })
      }
    },

    clamp(value: number, min: number, max: number) {
      return Math.min(Math.max(value, min), max)
    }
  }
})
</script>

<style scoped>
/* make items appear in one row */
.wrap {
  display: flex;
  height: 100%;
  padding: 1.5% 0 1.5% 0;
  flex-direction: column;
}

.wrap h4 {
  margin-top: -10px;
  margin-bottom: 10px;
}
</style>
