<template>
<div>
    <p>Auton</p>
    <DriveControls></DriveControls>
</div>
</template>

<script>
import ROSLIB from "roslib"
import DriveControls from "./DriveControls.vue"

export default {
  data() {
    return {
      count: 0
    }
  },

  created: function () {
  },

  components:{
    DriveControls,
}
}
</script>

