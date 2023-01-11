<template>
    <div class="wrap">
      <div class="buttons" v-for="j in 3">
        <template v-for="i in 3">
          <button class="cam_buttons" :class="{active_cam_button:camsEnabled[(i-1) + 3*(j-1)]}" v-on:click="$emit('cam_index', (i-1) + 3*(j-1))" :disabled="maxedOut && !camsEnabled[(i-1) + 3*(j-1)]"> <span>{{names[(i-1) + 3*(j-1)]}}</span> </button>
          <div class="fixed-spacer"></div>
        </template>
      </div>
    </div>
  </template>
  
  <script>
  
  export default {
    data() {
      return {
        opacities: new Array(9).fill(1.0)
      }
    },

    props: {
      capacity: {
        type: Number,
        required: true
      },
      camsEnabled: {
        type: Array,
        required: true
      },
      names: {
        type: Array,
        required: true
      }
    },
    
    computed: {
      maxedOut() {
        let num_enabled = 0
        for (let i = 0; i < this.camsEnabled.length; i++) {
          if(this.camsEnabled[i]) {
            num_enabled++;
          }
        }
        return num_enabled == this.capacity;
      }
    },

    watch: {
      camsEnabled: function() {
        for (let i = 0; i < this.camsEnabled.length; i++) {
          (this.camsEnabled[i]) ? this.opacities[i] = 0.55 : this.opacities[i] = 1.0;
        }
      }
    }

  }
  </script>
  
  <style scoped>
    .buttons {
      display: flex;
      align-items: center;
      justify-content: center;
      margin: 10px;
    }
    .cam_buttons {
      height:25px;
      width:100px;
      border: 1px solid black;
      border-radius: 5px;
    }
    .fixed-spacer {
      width:10px;
      height:auto;
    }

    .active_cam_button {
      background-color: green;
      color: white;
    }
  
  </style>