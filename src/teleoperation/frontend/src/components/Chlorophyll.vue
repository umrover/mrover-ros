<template>
  <div>
    <h3>Chlorophyll Test Control</h3>
    <div>
      <div>
        <ToggleButton id="white_led_toggle" :current-state="whiteLEDS_active" label-enable-text="White LEDs On"
          label-disable-text="White LEDs Off" @change="toggle_whiteLEDS()" />
        <ToggleButton id="uv_carousel" :current-state="UV_carousel" label-enable-text="Carousel UV On"
          label-disable-text="Carousel UV Off" @change="toggleCarouselUV()" />
      </div>
    </div>
    <div class="wrap-table">
      <div>
        <h3>Chlorophyll Spectral data</h3>
      </div>
      <table class="table table-bordered">
        <!-- <colgroup>
          <col style="width: 100px" />
          <col style="width: 63px" />
          <col style="width: 63px" />
          <col style="width: 63px" />
          <col style="width: 63px" />
          <col style="width: 63px" />
          <col style="width: 63px" />
          <col style="width: 63px" />
        </colgroup> -->
        <thead>
          <tr class="table-primary">
            <th>Wavelength</th>
            <th>610nm</th>
            <th>680nm</th>
            <th>730nm</th>
            <th>760nm</th>
            <th>810nm</th>
            <th>860nm</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="i in 3" :key="i">
            <td>
              Site {{ String.fromCharCode(64 + i) }}
            </td>
            <td v-if="spectral_data.length > 0 && !error[i - 1]" v-for="j in 6" :key="j">
              {{ spectral_data[i - 1][j - 1].toFixed(2) }}
            </td>
            <td v-else v-for="k in 6" :key="k">
              -
            </td>
          </tr>
        </tbody>
      </table>
    </div>
    <!-- <GenerateReport :spectral_data="spectral_data" /> -->
  </div>
</template>

<script lang="ts">
import ToggleButton from "./ToggleButton.vue";
// import ROSLIB from "roslib";
// import GenerateReport from "./GenerateReport.vue";
import { mapState } from 'vuex'

export default {
  components: {
    ToggleButton,
    // GenerateReport,
  },

  data() {
    return {
      whiteLEDS_active: false,
      spectral_data: [],
      error: []
    };
  },

  methods: {
    toggle_whiteLEDS: function () {
      // this.whiteLEDS_active = !this.whiteLEDS_active;
      // let whiteLedService = new ROSLIB.Service({
      //   ros: this.$ros,
      //   name: "enable_mosfet_device",
      //   serviceType: "mrover/EnableDevice",
      // });
      // let request = new ROSLIB.ServiceRequest({
      //   name: "white_led",
      //   enable: this.whiteLEDS_active,
      // });
      // whiteLedService.callService(request, (result) => {
      //   if (!result) {
      //     this.whiteLEDS_active = !this.whiteLEDS_active;
      //     alert("Toggling white LEDs failed.");
      //   }
      // });
    },

    toggleCarouselUV: function () {
      // this.UV_carousel = !this.UV_carousel;
      // let uvService = new ROSLIB.Service({
      //   ros: this.$ros,
      //   name: "enable_mosfet_device",
      //   serviceType: "mrover/EnableDevice",
      // });
      // let request = new ROSLIB.ServiceRequest({
      //   name: "uv_led_carousel",
      //   enable: this.UV_carousel,
      // });
      // uvService.callService(request, (result) => {
      //   if (!result) {
      //     this.UV_carousel = !this.UV_carousel;
      //     alert("Toggling Carousel UV failed.");
      //   }
      // });
    },
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == 'spectral_data') {
        this.spectral_data = msg.data
        this.error = msg.error
        console.log(this.spectral_data)
        console.log(this.error)
      }
    }
  }
};
</script>

<style scoped>
/*.box1 {
  text-align: left;
  vertical-align: top;
  display: inline-block;
}*/

.wrap-table {
  display: inline-block;
  align-content: center;
  height: max-content;
  padding-bottom: 5px;
}

.site {
  width: 100px;
}

.spectral-data {
  width: 63px;
}

/*.report {
  height: 5vh;
  align-items: center;
  padding-top: 3vh;
}

.tableFormat {
  border-collapse: collapse;
  border-spacing: 0;
}

.tableFormat td {
  border-color: black;
  border-style: solid;
  border-width: 1px;
  font-size: 14px;
  overflow: hidden;
  padding: 5px 5px;
  word-break: normal;
}

.tableFormat th {
  border-color: black;
  border-style: solid;
  border-width: 1px;
  font-size: 14px;
  font-weight: normal;
  overflow: hidden;
  padding: 10px 5px;
  word-break: normal;
}

.tableFormat .tableElement {
  border-color: inherit;
  text-align: center;
  vertical-align: top;
}*/
</style>