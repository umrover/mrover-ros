<template>
  <div class="wrap">
    <div class="page_header">
      <img
        src="/static/mrover_logo.png"
        alt="MRover"
        title="MRover"
        width="185"
        height="53"
      />
      <h1>Menu</h1>
    </div>

    <div class="content">
      <fieldset class="box row">
        <legend>Tasks</legend>
        <MenuButton link="#/EDMTask" name="EDM Mission" />
        <MenuButton link="#/ESTask" name="ES Mission" />
        <MenuButton link="#/ISHTask" name="ISH GUI" />
        <MenuButton link="#/SATask" name="Sample Acquisition GUI" />
        <MenuButton link="#/AutonTask" name="Autonomy Mission" />
        <MenuButton link="#/Control" name="Temp Controls GUI" />
        <MenuButton link="#/ROSDebug" name="ROS Debug Tools" />
      </fieldset>
    </div>
  </div>
</template>

<script>
import "../assets/style.css";
import MenuButton from "./MenuButton.vue";

export default {
  name: "Menu",

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

.content {
  margin: 10px;
}
</style>
