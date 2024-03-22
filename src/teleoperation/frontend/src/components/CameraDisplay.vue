<template>
    <div class="wrap">
        <div v-if="numStreams == 1">
            <CameraFeed :id="0" :port="0"></CameraFeed>
        </div>
        <div v-else class="grid-container">
            <div v-for="i in numStreams" :key="i" :class="'feed'+i">
                <CameraFeed :id="streamOrder[i-1]" :port="8080+i"></CameraFeed>
            </div>
        </div>
    </div>
  </template>
  
  <script lang="ts">
  import { defineComponent } from 'vue'
  import CameraFeed from '../components/CameraFeed.vue'
  
  export default defineComponent({
    components: {
        CameraFeed
    },
    props: {
        streamOrder: { //array of camera indices to stream. -1 indicates not used
            type: Array,
            required: true
        }
    },
    data() {
      return {
        numStreams: 0
      }
    },

    watch: {
        streamOrder: {
            handler: function() {
                this.numStreams = this.streamOrder.filter((num) => num != -1).length;
            },
            deep: true
        }
    },

    created: function () {
      
    }
  })
  </script>
  
  <style scoped>
    .grid-container {
        display: grid;
        grid-template-columns: repeat(2, 1fr);
        grid-template-rows: repeat(2, 1fr);
        grid-gap: 10px; 
        grid-template-areas: 'feed1 feed2'
                             'feed3 feed4';
    }

    .feed1 {
        grid-area: feed1;
    }
    .feed2 {
        grid-area: feed2;
    }
    .feed3 {
        grid-area: feed3;
    }
    .feed4 {
        grid-area: feed4;
    }
  </style>