<template>
<div>
  <div class="box page_header">
    <img src="/static/new_mrover.png" alt="MRover" title="MRover" width="185" height="53" />
    <h1>ISH</h1>
  </div>
    <div class="box camera light-bg">
      <Cameras v-bind:numCams="4" v-bind:primary="false"/>
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
 .box {
      border-radius: 5px;
      padding: 10px;
      border: 1px solid black;
    }

    .camera {
      width: 300px;
    }

    .light-bg {
        background-color: LightGrey;
    }


</style>

