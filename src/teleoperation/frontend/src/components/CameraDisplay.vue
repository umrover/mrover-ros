<template>
    <div class="wrap">
        <div class="grid-container">
            <div v-for="i in 4" :key="i" :class="'feed'+i">
                <div v-if="i <= numStreams">
                    <IKCameraFeed v-if="mission==='ik'" :id="streamOrder[i-1]"></IKCameraFeed>
                <SACameraFeed v-if="mission==='sa'" :id="streamOrder[i-1]"></SACameraFeed>
                <CameraFeed v-if="mission==='other'" :id="streamOrder[i-1]"></CameraFeed>
                </div>
            </div>
        </div>
    </div>
  </template>
  
  <script lang="ts">
  import { defineComponent } from 'vue'
  import IKCameraFeed from './IKCameraFeed.vue'
  import SACameraFeed from './SACameraFeed.vue'
  import CameraFeed from './CameraFeed.vue'
  
  export default defineComponent({
    components: {
        IKCameraFeed,
        SACameraFeed,
        CameraFeed
    },
    props: {
        streamOrder: { //array of camera indices to stream. -1 indicates not used
            type: Array,
            required: true
        },
        mission: {
            type: String, // {'sa', 'ik', 'other'}
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