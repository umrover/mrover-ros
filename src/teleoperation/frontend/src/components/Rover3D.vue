<template>
    <div class="wrap">
      <h3 class="header">Rover 3D</h3>
      <div id="threejs"></div>
    </div>
  </template>
  
  <script lang="ts">
  import { defineComponent } from 'vue'
  import { mapState } from 'vuex'
  import { threeSetup } from '../rover_three.js';
  
  export default defineComponent({
    data() {
      return {
        threeScene: null
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
           this.threeScene.updateJointAngles(msg.positions);
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