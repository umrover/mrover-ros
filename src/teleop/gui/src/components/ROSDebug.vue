<template>
  <div class="wrap">
    <div class="page_header">
      <h1>Debug Tools</h1>
      <img
        src="/static/mrover_logo.png"
        alt="MRover"
        title="MRover"
        width="185"
        height="53"
      />
    </div>
      <fieldset class="box row">
        <legend>Tasks</legend>
        <MenuButton link="#/ROSSend" name="ROS Send Message" />
        <MenuButton link="#/ROSEcho" name="ROS Echo Message" />
        <MenuButton link="#/ROSService" name="ROS Service" />
      </fieldset>
  </div>
</template>

<script>
import "../assets/style.css";
import MenuButton from "./MenuButton.vue";

export default {
  name: "Debug",

  components: {
    MenuButton,
  },
};
</script>

<style scoped>
.row {
  display: flex;
  margin: 5px;
}
</style>
