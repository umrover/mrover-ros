<template>
<<<<<<< HEAD
    <div class="wrap">
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
                    </tr>
                </thead>

                <tbody>
                    <tr v-for="(joint, index) in jointStateData.name" :key="index">
                        <td>{{ joint }}</td>
                        <td>
                            {{ (jointStateData.position[index] * radius_m).toFixed(3) }}
                        </td>
                        <td>
                            {{ (jointStateData.velocity[index] * radius_m).toFixed(3) }}
                        </td>
                        <td>
                            {{ (jointStateData.effort[index] * 1).toFixed(3) }}
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
                        <th v-for="name in jointStateData.name" :key="name">
                            {{ name.toFixed(3) }}
                        </th>
                    </tr>
                    <tr>
                        <th>Position (m)</th>
                        <td v-for="position in jointStateData.position" :key="position">
                            {{ (position * radius_m).toFixed(3) }}
                        </td>
                    </tr>
                    <tr>
                        <th>Velocity (m/s)</th>
                        <td v-for="velocity in jointStateData.velocity" :key="velocity">
                            {{ (velocity * radius_m).toFixed(3) }}
                        </td>
                    </tr>
                    <tr>
                        <th>Effort (Nm)</th>
                        <td v-for="effort in jointStateData.effort" :key="effort">
                            {{ effort.toFixed(3) }}
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
        jointStateData: {
            type: Object,
            required: true,
        },

        vertical: {
            type: Boolean,
            required: true,
        },
    },

    data() {
        return {
            radius_m: 1
        };
    },

    created: function () {
        // let radius_param = new ROSLIB.Param({
        //     ros: this.$ros,
        //     name: "wheel/radius"
        // });

        // radius_param.get((radius_m) => {
        //     this.radius_m = radius_m;
        // })
    }
};
</script>
  
<style scoped>
.wrap {
    display: inline-block;
    align-content: center;
}
</style>
=======
  <div class="wrap">
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
          </tr>
        </thead>

        <tbody>
          <tr v-for="(joint, index) in jointStateData.name" :key="index">
            <td>{{ joint }}</td>
            <td>
              {{ (jointStateData.position[index] * radius_m).toFixed(3) }}
            </td>
            <td>
              {{ (jointStateData.velocity[index] * radius_m).toFixed(3) }}
            </td>
            <td>
              {{ (jointStateData.effort[index] * 1).toFixed(3) }}
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
            <th v-for="name in jointStateData.name" :key="name">
              {{ name.toFixed(3) }}
            </th>
          </tr>
          <tr>
            <th>Position (m)</th>
            <td v-for="position in jointStateData.position" :key="position">
              {{ (position * radius_m).toFixed(3) }}
            </td>
          </tr>
          <tr>
            <th>Velocity (m/s)</th>
            <td v-for="velocity in jointStateData.velocity" :key="velocity">
              {{ (velocity * radius_m).toFixed(3) }}
            </td>
          </tr>
          <tr>
            <th>Effort (Nm)</th>
            <td v-for="effort in jointStateData.effort" :key="effort">
              {{ effort.toFixed(3) }}
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
    jointStateData: {
      type: Object,
      required: true
    },

    vertical: {
      type: Boolean,
      required: true
    }
  },

  data() {
    return {
      radius_m: 1
    }
  },

  created: function () {
    // let radius_param = new ROSLIB.Param({
    //     ros: this.$ros,
    //     name: "wheel/radius"
    // });
    // radius_param.get((radius_m) => {
    //     this.radius_m = radius_m;
    // })
  }
}
</script>

<style scoped>
.wrap {
  display: inline-block;
  align-content: center;
}
</style>
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
