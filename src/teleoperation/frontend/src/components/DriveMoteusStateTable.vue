<template>
  <div class="wrap">
    <div>
      <h3>Drive Moteus States</h3>
    </div>
    <table class="table">
      <thead>
        <tr class="table-primary">
          <th></th>
          <th>Left</th>
          <th>Right</th>
        </tr>
      </thead>
      <tbody>
        <template v-for="i in 3" :key="i">
          <tr>
            <th class="table-secondary">Name</th>
            <td>
              {{ moteusStateData.name[(i-1) * 2] }}
            </td>
            <td>
              {{ moteusStateData.name[(i-1) * 2 + 1] }}
            </td>
          </tr>
          <tr>
            <th class="table-secondary">State</th>
            <td>
              {{ moteusStateData.state[(i-1) * 2] }}
            </td>
            <td>
              {{ moteusStateData.state[(i-1) * 2 + 1] }}
            </td>
          </tr>
          <tr>
            <th class="table-secondary">Error</th>
            <td>
              {{ moteusStateData.error[(i-1) * 2] }}
            </td>
            <td>
              {{ moteusStateData.error[(i-1) * 2 + 1] }}
            </td>
          </tr>
          <tr class="bold-border">
            <th class="table-secondary">Limit Hit</th>
            <td>
              {{ moteusStateData.limit_hit[(i-1) * 2] }}
            </td>
            <td>
              {{ moteusStateData.limit_hit[(i-1) * 2 + 1] }}
            </td>
          </tr>
        </template>
      </tbody>
    </table>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'

export default defineComponent({
  props: {
    moteusStateData: {
      type: Object,
      required: true
    }
  }
})
</script>

<style scoped>
.wrap {
  display: inline-block;
  align-content: center;
}

.bold-border {
  border-bottom: 1px solid #000;
}
</style>
