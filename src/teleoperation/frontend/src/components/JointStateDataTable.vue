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
          <th class='table-secondary'>Pos.</th>
          <td v-for='(pos, i) in position' :key='i'>
            {{ pos }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary'>Vel.</th>
          <td v-for='(vel, i) in velocity' :key='i'>
            {{ vel }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary'>Effort</th>
          <td v-for='(eff, i) in effort' :key='i'>
            {{ eff }}
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
        position: [] as number[],
        velocity: [] as number[],
        effort: [] as number[]
      }
    },
  
    computed: {
      ...mapState('websocket', ['message'])
    },
  
    watch: {
      message(msg) {
        if (msg.type == this.msgType) {
          this.name = msg.name
          this.position = msg.position
          this.velocity = msg.velocity
          this.effor = msg.effort
        }
      }
    }
  })
  </script>
  
  <style scoped>
  .wrap {
    display: inline-block;
    align-content: center;
    margin: 0px 10px 0px 10px;
  }
  </style>