<template>
  <div class="wrap">
    <div>
      <h3>{{ header }}</h3>
    </div>
    <table class="table table-bordered" style="table-layout: fixed; width: auto">
      <tbody>
        <tr>
          <th class="table-secondary">Motor</th>
          <td v-for="(name, i) in moteusStateName" :key="i">
            {{ name }}
          </td>
        </tr>
        <tr>
          <th class="table-secondary">State</th>
          <td v-for="(state, i) in moteusStateState" :key="i">
            {{ state }}
          </td>
        </tr>
        <tr>
          <th class="table-secondary">Error</th>
          <td v-for="(error, i) in moteusStateError" :key="i">
            {{ error }}
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
      moteusStateName: [] as string[],
      moteusStateState: [] as string[],
      moteusStateError: [] as string[]
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == 'arm_moteus') {
        let index = this.moteusStateName.findIndex((n) => n === msg.name)

        if (this.moteusStateName.length == 4 || index != -1) {
          //if all joints are in table or there's an update to one before all are in
          this.update(msg, index)
        } else {
          this.moteusStateName.push(msg.name)
          this.moteusStateState.push(msg.state)
          this.moteusStateError.push(msg.error)
        }
      }
    }
  },

  methods: {
    update(msg: { name: string; state: string; error: string }, index: number) {
      if (index !== -1) {
        this.moteusStateState[index] = msg.state
        this.moteusStateError[index] = msg.error
      } else {
        console.log('Invalid arm moteus name: ' + msg.name)
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
