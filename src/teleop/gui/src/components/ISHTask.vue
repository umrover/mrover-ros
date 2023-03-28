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
      <h1>ISH Dashboard</h1>
      <div class="spacer"></div>
      <div class="spacer"></div>
      <CommReadout class="comm"></CommReadout>
      <div class="help">
        <img
          src="/static/help.png"
          alt="Help"
          title="Help"
          width="48"
          height="48"
        />
      </div>
      <div class="helpscreen"></div>
      <div
        class="helpimages"
        style="
          display: flex;
          align-items: center;
          justify-content: space-evenly;
        "
      >
        <img
          src="/static/joystick.png"
          alt="Joystick"
          title="Joystick Controls"
          style="width: auto; height: 70%; display: inline-block"
        />
      </div>
    </div>
    <div class="box light-bg siteSelect">
      <SelectSite @site="onSiteChange" />
    </div>
    <div class="box light-bg raman">
      <Raman />
    </div>
    <div class="box light-bg sudan">
      <Sudan :site="site" :site-index="siteIndex" />
    </div>
    <div class="box light-bg cameras">
      <Cameras :primary="primary" />
    </div>
    <div class="box light-bg carousel">
      <Carousel />
    </div>
    <div class="box light-bg cache">
      <Cache :site="site" />
    </div>
    <div class="box light-bg chlorophyll">
      <Chlorophyll :spectral_data="spectral_data" />
    </div>
    <div class="box light-bg amino">
      <Amino :site="site" :site-index="siteIndex" />
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import SelectSite from "./SelectSite.vue";
import Raman from "./Raman.vue";
import Sudan from "./Sudan.vue";
import Carousel from "./Carousel.vue";
import Cache from "./Cache.vue";
import Chlorophyll from "./Chlorophyll.vue";
import Amino from "./Amino.vue";
import Cameras from "../components/Cameras.vue";
import CommReadout from "./CommReadout.vue";

export default {
  components: {
    SelectSite,
    Raman,
    Sudan,
    Carousel,
    Cache,
    Chlorophyll,
    Amino,
    Cameras,
    CommReadout,
  },
  data() {
    return {
      site: "A",

      // Initialize this so that computed property won't be mad
      siteIndexMapping: { A: 0, B: 1, C: 2 },

      spectral_data: [0, 0, 0, 0, 0, 0],

      //Data Subscribers
      spectral_sub: null,

      primary: false,
    };
  },
  computed: {
    siteIndex: function () {
      // Return the indice for the specified site
      return this.siteIndexMapping[this.site];
    },
  },

  created: function () {
    this.spectral_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "science/spectral",
      messageType: "mrover/Spectral",
    });

    this.spectral_sub.subscribe((msg) => {
      // Callback for spectral_sub
      this.spectral_data = msg.data;
    });

    // Get carousel site index mappings
    let mapping = new ROSLIB.Param({
      ros: this.$ros,
      name: "teleop/carousel_site_mappings",
    });

    mapping.get((param) => {
      this.siteIndexMapping = param;
    });
  },

  methods: {
    onSiteChange(value) {
      this.site = value;
    },
  },
};
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: 85vh auto auto;
  grid-template-rows: 60px auto auto auto auto auto auto auto auto auto;
  grid-template-areas:
    "header header header"
    "cameras cameras siteSelect"
    "cameras cameras raman"
    "cameras cameras sudan"
    "carousel chlorophyll chlorophyll"
    "carousel chlorophyll chlorophyll"
    "cache chlorophyll chlorophyll"
    "cache chlorophyll chlorophyll"
    "cache amino amino"
    "cache amino amino";
  font-family: sans-serif;
  height: auto;
}

img {
  border: none;
  border-radius: 0px;
}

.spacer {
  flex-grow: 0.8;
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

.box {
  border-radius: 5px;
  padding: 10px;
  border: 1px solid black;
}

.light-bg {
  background-color: LightGrey;
}

.header {
  grid-area: header;
  display: flex;
  align-items: center;
}

.header h1 {
  margin-left: 5px;
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

.sudan {
  grid-area: sudan;
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
