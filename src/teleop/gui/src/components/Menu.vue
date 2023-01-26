<template>
  <div class="wrap">
    <div class="box page_header">
      <img src="/static/new_mrover.png" alt="MRover" title="MRover" width="185" height="53" />
      <h1>Menu</h1>
      <div class="spacer"></div>
    </div>

    <div class="content">
      <fieldset class="box row">
        <legend>Tasks</legend>
        <MenuButton link="#/ERDTask" name="ERD Mission" />
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
import MenuButton from './MenuButton.vue'

export default {
  name: 'Menu',

  components: {
    MenuButton
  }
}
</script>

<style scoped>

.pages {
  grid-area: pages;
  border: black solid 1px;
  border-radius: 5px;
  background-color: lightgray;
}

.row {
  display: flex;
  margin: 5px;
}

.wrap {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: 1fr;
  grid-template-rows: 60px 1fr;
  grid-template-areas: "header" "pages";
  font-family: sans-serif;
  height: auto;
}

.content {
  margin: 10px;
}

</style>
