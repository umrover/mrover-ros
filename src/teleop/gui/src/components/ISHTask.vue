<template>
  <div>
    <p>ISH</p>
    <div class="box camera light-bg">
      <Cameras :num-cams="4" :primary="false" />
    </div>
  </div>
</template>

<script>
import Cameras from "../components/Cameras.vue";

export default {
  components: {
    Cameras,
  },
  data() {
    return {};
  },

  methods: {
    publish: function () {},
  },
};
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
