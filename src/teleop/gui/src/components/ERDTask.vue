<template>
<div>
    <p>ERD</p>
    <ArmControls></ArmControls>
</div>
</template>

<script>
import ROSLIB from "roslib"
import ArmControls from "./ArmControls.vue";

export default {
  data() {
    return {
      count: 0
    }
  },

  created: function () {
  },

  components:{
    ArmControls
}
}
</script>