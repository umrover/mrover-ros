<template>
  <div class="wrap">
    <div>
      <h3>{{ header }}</h3>
    </div>
    <table class="table table-bordered" style="table-layout: fixed; width: auto">
      <tbody>
        <tr>
          <th class="table-secondary">Motor</th>
          <td v-for="(name, i) in moteusName" :key="i">
            {{ name }}
          </td>
        </tr>
        <tr>
          <th class="table-secondary">State</th>
          <td v-for="(state, i) in moteusState" :key="i">
            {{ state }}
          </td>
        </tr>
        <tr>
          <th class="table-secondary">Error</th>
          <td v-for="(error, i) in moteusError" :key="i">
            {{ error }}
          </td>
        </tr>
        <tr>
          <th class="table-secondary">Limit Hits</th>
          <td v-for="(limits, i) in moteusLimits" :key="i">
            {{ limits }}
          </td>
        </tr>
      </tbody>
    </table>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import { mapState } from 'vuex'

export default defineComponent({
  props: {
    header: {
      type: String,
      required: false,
      default: 'Arm Moteus States'
    }
  },

  data() {
    return {
      moteusName: [] as string[],
      moteusState: [] as string[],
      moteusError: [] as string[],
      moteusLimits: [] as boolean[]
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == 'arm_moteus') {
        this.moteusName = msg.name
        this.moteusState = msg.state
        this.moteusError = msg.error
        this.moteusLimits = msg.limit_hit
      }
    }
  },
})
</script>

<style scoped>
.wrap {
  display: inline-block;
  align-content: center;
}
</style>
