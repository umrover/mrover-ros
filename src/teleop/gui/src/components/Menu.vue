<template>
  <div class="wrapper">
    <div class="box header">
      <img
        src="/static/mrover.png"
        alt="MRover"
        title="MRover"
        width="48"
        height="48"
      />
      <h1>Menu</h1>
      <div class="spacer"></div>
      <div class="mode-toggle">
        <ToggleButton
          id="competition_mode"
          :label-enable-text="'Competition Mode On'"
          :label-disable-text="'Competition Mode Off'"
          :current-state="competitionMode"
          @change="toggleCompetitionMode()"
        ></ToggleButton>
      </div>
    </div>

    <div class="pages">
      <fieldset class="tasks row">
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
import MenuButton from "./MenuButton.vue";
import ToggleButton from "./ToggleButton.vue";
import Vue from "vue";
import ROSLIB from "roslib";
import { disableAutonLED } from "../utils.js";

export default {
  name: "MainMenu",
  components: {
    MenuButton,
    ToggleButton,
  },

  data() {
    return {
      competitionMode: false,
    };
  },

  watch: {
    competitionMode: function (val) {
      if (val) {
        Vue.prototype.$ros = new ROSLIB.Ros({
          url: "ws://10.0.0.7:9090",
        });
      } else {
        Vue.prototype.$ros = new ROSLIB.Ros({
          url: "ws://localhost:9090",
        });
      }
    },
  },

  created: function () {
    disableAutonLED(this.$ros);
    this.competitionMode = Vue.prototype.$competitionMode;
  },

  methods: {
    toggleCompetitionMode() {
      this.competitionMode = !this.competitionMode;
      Vue.prototype.$competitionMode = this.competitionMode;
    },
  },
};
</script>

<style scoped>
.box {
  border-radius: 5px;
  padding: 10px;
  border: 1px solid black;
}

.header {
  grid-area: header;
  display: flex;
  align-items: center;
}

.header h1 {
  margin-left: 5px;
}

img {
  border: none;
  border-radius: 0px;
}

.pages {
  grid-area: pages;
  border: black solid 1px;
  border-radius: 5px;
  background-color: lightgray;
}

.row {
  display: flex;
  border: black solid 1px;
  border-radius: 5px;
  background-color: darkgray;
  margin: 5px;
}

.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: 1fr;
  grid-template-rows: 60px 1fr;
  grid-template-areas: "header" "pages";
  font-family: sans-serif;
  height: auto;
}

.mode-toggle {
  margin-left: auto;
}
</style>
