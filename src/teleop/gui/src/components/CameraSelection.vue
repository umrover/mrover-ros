<template>
    <div class="wrap2">
      <!-- <span v-if="cam_index > -1" class="title">Current Camera: {{cam_index}}</span> -->
      <div class="buttons">
        <template v-for="i in 3">
          <button class="cam_buttons" v-bind:style="{opacity: opacities[i-1]}" v-on:click="$emit('cam_index', i-1)" :disabled="maxedOut && !camsEnabled[i-1]"> <span>{{names[i-1]}}</span> </button>
          <div class="fixed-spacer"></div>
        </template>
      </div>
      <div class="buttons">
        <template v-for="i in 3">
          <button class="cam_buttons" v-bind:style="{opacity: opacities[i+2]}" v-on:click="$emit('cam_index', i+2)" :disabled="maxedOut && !camsEnabled[i+2]"> <span>{{names[i+2]}}</span> </button>
          <div class="fixed-spacer"></div>
        </template>
      </div>
      <div class="buttons">
        <template v-for="i in 3">
          <button class="cam_buttons" v-bind:style="{opacity: opacities[i+5]}" v-on:click="$emit('cam_index', i+5)" :disabled="maxedOut && !camsEnabled[i+5]"> <span>{{names[i+5]}}</span> </button>
          <div class="fixed-spacer"></div>
        </template>
      </div>
    </div>
  </template>
  
  <script>
    export default {
      data() {
        return {
          opacities: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        }
      },
      props: {
        numCams: {
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
        maxedOut: function() {
          let num_enabled = 0
          for (let i = 0; i < this.camsEnabled.length; i++) {
            if (this.camsEnabled[i]) {
              num_enabled++
            }
          }
          return num_enabled == this.numCams
        }
      },
      watch: {
        camsEnabled() {
          for (let i = 0; i < this.camsEnabled.length; i++) {
            if (this.camsEnabled[i]) {
              this.opacities[i] = 0.55
            }
            else {
              this.opacities[i] = 1.0
            }
          }
        }
      }
    }
  </script>
  
  <style>
    .title {
      grid-area: title;
      margin: auto;
      width: auto;
    }
    .buttons {
      display: flex;
      align-items: center;
      justify-content: center;
      grid-area: buttons;
    }
    .cam_buttons {
      height:20px;
      width:100px;
    }
    .fixed-spacer {
      width:10px;
      height:auto;
    }
  </style>