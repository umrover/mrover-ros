<template>
  <div class="wrap mx-2">
    <div v-if="vertical">
      <div>
        <h3>Motor Data</h3>
      </div>
      <table class="table">
        <thead>
          <tr class="table-primary">
            <th>Motor</th>
            <th>Positon (m)</th>
            <th>Velocity (m/s)</th>
            <th>Effort (Nm)</th>
            <th>State</th>
            <th>Error</th>
          </tr>
        </thead>

        <tbody>
          <tr v-for="(joint, index) in motorData.name" :key="index">
            <td>{{ joint }}</td>
            <td>
              {{ motorData.position[index].toFixed(3) }}
            </td>
            <td>
              {{ motorData.velocity[index].toFixed(3) }}
            </td>
            <td>
              {{ motorData.effort[index].toFixed(3) }}
            </td>
            <td>
              {{ motorData.state[index] }}
            </td>
            <td>
              {{ motorData.error[index] }}
            </td>
          </tr>
        </tbody>
      </table>
    </div>

    <div v-else>
      <div>
        <h3>Motor Data</h3>
      </div>
      <table class="table">
        <thead>
          <tr>
            <th>Motor</th>
            <th v-for="name in motorData.name" :key="name">
              {{ name.toFixed(3) }}
            </th>
          </tr>
          <tr>
            <th>Position (m)</th>
            <td v-for="position in motorData.position" :key="position">
              {{ position.toFixed(3) }}
            </td>
          </tr>
          <tr>
            <th>Velocity (m/s)</th>
            <td v-for="velocity in motorData.velocity" :key="velocity">
              {{ velocity.toFixed(3) }}
            </td>
          </tr>
          <tr>
            <th>Effort (Nm)</th>
            <td v-for="effort in motorData.effort" :key="effort">
              {{ effort.toFixed(3) }}
            </td>
          </tr>
          <tr>
            <th>State</th>
            <td v-for="state in motorData.state" :key="state">
              {{ state }}
            </td>
          </tr>
          <tr>
            <th>Error</th>
            <td v-for="error in motorData.error" :key="error">
              {{ error }}
            </td>
          </tr>
        </thead>
      </table>
    </div>
  </div>
</template>

<script lang="ts">
export default {
  props: {
    // Table will only render headers if these values are not passed w/ v-bind
    motorData: {
      type: Object,
      required: true
    },

    vertical: {
      type: Boolean,
      required: true
    }
  },

  data() {
    return {}
  }
}
</script>

<style scoped>
.wrap {
  display: inline-block;
  align-content: center;
}
</style>
