<template>
<div class="wrap">
    <div class="page_header">
    <h1>Debug Tools</h1>
    <img src="/static/new_mrover.png" alt="MRover" title="MRover" width="185" height="53" />
    </div>

    <div class="pages">
      <fieldset class="tasks row">
        <legend>Tools</legend>
        <MenuButton link="#/ROSSend" name="ROS Send Message" />
        <MenuButton link="#/ROSEcho" name="ROS Echo Message" />
        <MenuButton link="#/ROSService" name="ROS Service" />
      </fieldset>
    </div>
  </div>
</template>

<script>
import '../assets/style.css';
import MenuButton from "./MenuButton.vue";

export default {
  name: "Debug",

  components: {
    MenuButton,
  },
};
</script>

<style scoped>

.pages {
grid-area: pages;
}

.row {
display: flex;
margin: 5px;
}

.wrap {
/* display: grid;
grid-gap: 10px;
grid-template-columns: 1fr;
grid-template-rows: 60px 1fr;
grid-template-areas: "header" "pages";
font-family: sans-serif;
height: auto; */
}
</style>
