<template>
  <div class="wrapper">
    <div class="shadow p-3 mb-5 header">
      <h1>Menu</h1>
      <img
        src="../static/mrover.png"
        alt="MRover"
        title="MRover"
        width="200"
      />
    </div>

    <div class="pages">
      <fieldset class="row">
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

<script lang="ts">
import MenuButton from "./MenuButton.vue";

import { defineComponent, inject } from 'vue'

export default defineComponent({
  name: "MainMenu",
  components: {
    MenuButton,
  },
  created() {
    const ws:any = inject<WebSocket>('webSocketService');

    ws.send('Hello, WebSocket!');
  },

  data() {
    return {};
  }

});
</script>

<style scoped>

.header {
  grid-area: header;
  display: flex;
  align-items: center;
  justify-content: center;
}

.header h1 {
  position: absolute;
  left: 25px;
  top: 10px;
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
  grid-template-rows: 125px 1fr;
  grid-template-areas: "header" "pages";
  font-family: sans-serif;
  height: auto;
}
</style>