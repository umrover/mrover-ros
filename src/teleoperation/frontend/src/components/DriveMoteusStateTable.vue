<template>
<<<<<<< HEAD
    <div class="wrap">
      <div>
        <h3>Drive Moteus States</h3>
      </div>
      <table
        class="table"
        style="table-layout: fixed; width: 400px"
      >
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
                    {{ moteusStateData.name[i*2] }}
                    </td>
                    <td>
                    {{ moteusStateData.name[i*2+1] }}
                    </td>
                </tr>
                <tr>
                    <th class="table-secondary">State</th>
                    <td>
                    {{ moteusStateData.state[i*2] }}
                    </td>
                    <td>
                    {{ moteusStateData.state[i*2+1] }}
                    </td>
                </tr>
                <tr class="bold-border">
                    <th class="table-secondary">Error</th>
                    <td>
                    {{ moteusStateData.error[i*2] }}
                    </td>
                    <td>
                    {{ moteusStateData.error[i*2+1] }}
                    </td>
                </tr>
            </template>
        </tbody>
      </table>
    </div>
  </template>
  
  <script lang="ts">
  import { defineComponent } from 'vue';

  
  export default defineComponent({
    props: {
      moteusStateData: {
        type: Object,
        required: true,
      },
    },
  });
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
=======
  <div class="wrap">
    <div>
      <h3>Drive Moteus States</h3>
    </div>
    <table class="table" style="table-layout: fixed; width: 400px">
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
              {{ moteusStateData.name[i * 2] }}
            </td>
            <td>
              {{ moteusStateData.name[i * 2 + 1] }}
            </td>
          </tr>
          <tr>
            <th class="table-secondary">State</th>
            <td>
              {{ moteusStateData.state[i * 2] }}
            </td>
            <td>
              {{ moteusStateData.state[i * 2 + 1] }}
            </td>
          </tr>
          <tr class="bold-border">
            <th class="table-secondary">Error</th>
            <td>
              {{ moteusStateData.error[i * 2] }}
            </td>
            <td>
              {{ moteusStateData.error[i * 2 + 1] }}
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
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
