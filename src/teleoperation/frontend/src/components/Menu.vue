<template>
  <div class="wrapper">
    <div class="shadow p-3 mb-5 header">
      <h1>Menu</h1>
      <img class="logo" src="/mrover.png" alt="MRover" title="MRover" width="200" />
    </div>

    <div class="pages">
      <fieldset class="row">
        <legend>Tasks</legend>
        <MenuButton link="/DMTask" name="DM Mission" />
        <MenuButton link="/ESTask" name="ES Mission" />
        <MenuButton link="/ISHTask" name="ISH GUI" />
        <MenuButton link="/SATask" name="Sample Acquisition GUI" />
        <MenuButton link="/AutonTask" name="Autonomy Mission" />
        <MenuButton link="/Cameras" name="Cameras" />
      </fieldset>
    </div>
  </div>
</template>

<script lang="ts">
import MenuButton from './MenuButton.vue'
export default {
  components: {
    MenuButton
  }
}
</script>
<style scoped>
.header {
  grid-area: header;
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px;
}

.logo {
  position: absolute;
  left: 50%;
  transform: translateX(-50%);
}

.pages {
  grid-area: pages;
}

.row {
  display: flex;
  justify-content: center;
  text-align: center;
  margin: 5px;
}

.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: 1fr;
  grid-template-rows: auto 1fr;
  grid-template-areas: 'header' 'pages';
  font-family: sans-serif;
  height: auto;
}
</style>
