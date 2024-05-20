<template>
  <div class='wrap'>
    <div>
      <h3>{{ header }}</h3>
    </div>
    <table class='table table-bordered' style='table-layout: fixed; width: auto'>
      <tbody>
      <tr>
        <th class='table-secondary'>Motor</th>
        <td v-for='(name, i) in name' :key='i'>
          {{ name }}
        </td>
      </tr>
      <tr>
        <th class='table-secondary'>State</th>
        <td v-for='(state, i) in state' :key='i'>
          {{ state }}
        </td>
      </tr>
      <tr>
        <th class='table-secondary'>Error</th>
        <td v-for='(error, i) in error' :key='i'>
          {{ error }}
        </td>
      </tr>
      <tr>
        <th class='table-secondary'>Limit Hits</th>
        <td v-for='(limits, i) in limits' :key='i'>
          {{ limits }}
        </td>
      </tr>
      </tbody>
    </table>
  </div>
</template>

<script lang='ts'>
import { defineComponent } from 'vue'
import { mapState } from 'vuex'

export default defineComponent({
  props: {
    header: {
      type: String,
      required: true,
    },
    msgType: {
      type: String,
      required: true,
    }
  },

  data() {
    return {
      name: [] as string[],
      state: [] as string[],
      error: [] as string[],
      limits: [] as boolean[]
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == this.msgType) {
        this.name = msg.name
        this.state = msg.state
        this.error = msg.error
        this.limits = msg.limit_hit
      }
    }
  }
})
</script>

<style scoped>
.wrap {
  display: inline-block;
  align-content: center;
}
</style>
