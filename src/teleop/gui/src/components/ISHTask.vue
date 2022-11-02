<template>
<div>
    <p>ISH</p>
    <div class="box camera">
      <Cameras v-bind:numCams="4" v-bind:primary="true" v-bind:mission="'Science'" v-bind:channel="'/cameras_control_ish'"/>
    </div>
</div>
</template>

<script>

import Cameras from '../components/Cameras.vue';

export default {
  data() {
    return {

    }
  },

  methods: {
    publish : function(){

    }
  },

  components:{
    Cameras
}
}
</script>

<style>

  :root {
    --background: white;
    --secondary-color: rgb(132, 169, 224);
    --shadow-color: rgba(236, 236, 236, 0.966);
  }
 .box {
        border-radius: 5px;
        padding: 10px;
        border: 1px solid var(--shadow-color);
        box-shadow: 2px 2px 15px var(--shadow-color), -2px -2px 15px var(--shadow-color);
    }

    .camera {
      width: 300px;
    }


</style>

