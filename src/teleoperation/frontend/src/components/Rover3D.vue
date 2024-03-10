<template>
    <div class="wrap">
      <h3 class="header">Rover 3D</h3>
      <div id="threejs"></div>

      <div class="col" v-for="(joint, i) in temp_positions" :key="i">
        <label>{{ joint }}</label>
        <input
          class="form-control"
          type="number"
          v-model="positions[i]"
        />
      </div>
      <div class="col text-center">
        <button class="btn btn-primary" @click="threeScene.fk(positions)">Submit</button>
      </div>
    </div>
  </template>
  
  <script lang="ts">
  import { defineComponent } from 'vue'
  import { mapState } from 'vuex'
  import { threeSetup } from '../rover_three.js';
  
  export default defineComponent({
    data() {
      return {
        threeScene: null,
        temp_positions: ["base", "a", "b", "c", "d", "e"],
        positions: []
      }
    },
 
    mounted() {
        this.threeScene = threeSetup("threejs");
    },

    computed: {
      ...mapState('websocket', ['message']),
    },

    watch: {
        message(msg) {
          if(msg.type == "fk") {
            this.threeScene.fk(msg.positions);
          }
          else if(msg.type == "ik") {
            this.threeScene.ik(msg.target);
          }
        }
    }
  })
  </script>
  
  <style scoped>
  .wrap {
    margin: 5px;
    border: 1px solid black;
  }
  
  .header {
    text-align: center;
  }

  #threejs {
    width: 600px;
    height: 600px;
  }
  </style>