<template>
  <div class="wrapper">
    <div class="shadow p-3 mb-5 header">
      <h1>ISH Dashboard</h1>
      <!-- <MCUReset class="mcu_reset"></MCUReset>
        <CommReadout class="comms"></CommReadout> -->
      <img class="logo" src="/mrover.png" alt="MRover" title="MRover" width="200" />
      <div class="help">
        <img src="help.png" alt="Help" title="Help" width="48" height="48" />
      </div>
      <div class="helpscreen"></div>
      <div
        class="helpimages"
        style="display: flex; align-items: center; justify-content: space-evenly"
      >
        <img
          src="joystick.png"
          alt="Joystick"
          title="Joystick Controls"
          style="width: auto; height: 70%; display: inline-block"
        />
      </div>
    </div>
    <!-- <div class="shadow p-3 rounded siteSelect">
        <SelectSite @site="onSiteChange" />
      </div> -->
    <!-- <div class="shadow p-3 rounded raman">
        <Raman />
      </div> -->
    <!-- ADD BENEDICTS TEST HERE-->
    <div class="shadow p-3 rounded cameras">
      <Cameras :primary="primary" />
    </div>
    <!-- <div class="shadow p-3 rounded cache">
        <Cache />
      </div> -->
    <!-- <div class="shadow p-3 rounded chlorophyll">
        <Chlorophyll :spectral_data="spectral_data" />
      </div> -->
    <!-- <div class="shadow p-3 rounded amino">
        <Amino :site="site" :site-index="siteIndex" />
      </div> -->
  </div>
</template>

<script lang="ts">
//   import SelectSite from "./SelectSite.vue";
//   import Raman from "./Raman.vue";
//   import Cache from "./Cache.vue";
//   import Chlorophyll from "./Chlorophyll.vue";
//   import Amino from "./Amino.vue";
import Cameras from '../components/Cameras.vue'
//   import CommReadout from "./CommReadout.vue";
//   import MCUReset from "./MCUReset.vue"
import { disableAutonLED } from '../utils.js'

type StringIntDictionary = {
  [key: string]: number
}

export default {
  components: {
    //   SelectSite,
    //   Raman,
    //   Cache,
    //   Chlorophyll,
    //   Amino,
    Cameras
    //   CommReadout,
    //   MCUReset,
  },
  data() {
    return {
      site: 'A' as string,

      // Initialize this so that computed property won't be mad
      siteIndexMapping: { A: 0, B: 1, C: 2 } as StringIntDictionary,

      spectral_data: [0, 0, 0, 0, 0, 0],

      primary: false
    }
  },
  computed: {
    siteIndex: function () {
      // Return the indice for the specified site
      return this.siteIndexMapping[this.site]
    }
  },

  created: function () {
    disableAutonLED()
    //   this.spectral_sub = new ROSLIB.Topic({
    //     ros: this.$ros,
    //     name: "science/spectral",
    //     messageType: "mrover/Spectral",
    //   });

    //   this.spectral_sub.subscribe((msg) => {
    //     // Callback for spectral_sub
    //     this.spectral_data = msg.data;
    //   });

    //   // Get carousel site index mappings
    //   let mapping = new ROSLIB.Param({
    //     ros: this.$ros,
    //     name: "teleop/site_mappings",
    //   });

    //   mapping.get((param) => {
    //     this.siteIndexMapping = param;
    //   });
  },

  methods: {
    onSiteChange(value: String) {
      this.site = value
    }
  }
}
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: repeat(2, auto);
  grid-template-rows: repeat(7, auto);
  grid-template-areas:
    'header header'
    'cameras siteSelect'
    'cameras raman'
    'cameras chlorophyll'
    'carousel chlorophyll'
    'carousel amino'
    'cache amino';
  font-family: sans-serif;
  height: auto;
}

.comms {
  margin-right: 5px;
}

.helpscreen {
  z-index: 1000000000;
  display: block;
  visibility: hidden;
  background-color: black;
  opacity: 0.8;
  position: absolute;
  left: 0px;
  top: 0px;
  width: 100%;
  height: 100%;
}

.helpimages {
  z-index: 1000000001;
  visibility: hidden;
  position: absolute;
  left: 5%;
  top: 5%;
  width: 90%;
  height: 90%;
}

.help {
  z-index: 1000000002;
  display: flex;
  float: right;
  opacity: 0.8;
  cursor: auto;
}

.help:hover {
  opacity: 1;
  cursor: pointer;
}

.help:hover ~ .helpscreen,
.help:hover ~ .helpimages {
  visibility: visible;
}

.header {
  grid-area: header;
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px;
}

.cameras {
  grid-area: cameras;
}

.siteSelect {
  grid-area: siteSelect;
}

.raman {
  grid-area: raman;
}

.carousel {
  grid-area: carousel;
}

.chlorophyll {
  grid-area: chlorophyll;
}

.cache {
  grid-area: cache;
}

.amino {
  grid-area: amino;
}
</style>
