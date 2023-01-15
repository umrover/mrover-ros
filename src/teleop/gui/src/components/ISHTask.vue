<template>
<div>
    <p>ISH</p>
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

