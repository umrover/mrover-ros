<template>
  <div>
    <h3>Chlorophyll Test Control</h3>
    <div>
      <div>
        <ToggleButton
          id="white_led_toggle"
          :current-state="whiteLEDS_active"
          label-enable-text="White LEDs On"
          label-disable-text="White LEDs Off"
          @change="toggle_whiteLEDS()"
        />
        <ToggleButton
          id="uv_carousel"
          :current-state="UV_carousel"
          label-enable-text="Carousel UV On"
          label-disable-text="Carousel UV Off"
          @change="toggleCarouselUV()"
        />
      </div>
    </div>
    <div class="wrap-table">
      <div>
        <h3>Chlorophyll Spectral data</h3>
      </div>
      <table
        class="tableFormat"
        style="undefined;table-layout: fixed; width: 434px"
      >
        <colgroup>
          <col style="width: 100px" />
          <col style="width: 63px" />
          <col style="width: 63px" />
          <col style="width: 63px" />
          <col style="width: 63px" />
          <col style="width: 63px" />
          <col style="width: 63px" />
          <col style="width: 63px" />
        </colgroup>
        <thead>
          <tr>
            <th class="tableElement">Wavelength</th>
            <th class="tableElement">610nm</th>
            <th class="tableElement">680nm</th>
            <th class="tableElement">730nm</th>
            <th class="tableElement">760nm</th>
            <th class="tableElement">810nm</th>
            <th class="tableElement">860nm</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td class="tableElement">Spec 0</td>
            <td v-for="i in 6" :key="i" class="tableElement">
              {{ spectral_data[i - 1].toFixed(0) }}
            </td>
          </tr>
        </tbody>
      </table>
    </div>
    <GenerateReport :spectral_data="spectral_data" />
  </div>
</template>

<script>
import ToggleButton from "./ToggleButton.vue";
import ROSLIB from "roslib";
import GenerateReport from "./GenerateReport.vue";

export default {
  components: {
    ToggleButton,
    GenerateReport,
  },

  props: {
    spectral_data: {
      type: Array,
      required: true,
    },
  },
  data() {
    return {
      whiteLEDS_active: false,
      UV_carousel: false,
    };
  },

  methods: {
    toggle_whiteLEDS: function () {
      this.whiteLEDS_active = !this.whiteLEDS_active;
      let whiteLedService = new ROSLIB.Service({
        ros: this.$ros,
        name: "enable_mosfet_device",
        serviceType: "mrover/EnableDevice",
      });
      let request = new ROSLIB.ServiceRequest({
        name: "white_led",
        enable: this.whiteLEDS_active,
      });
      whiteLedService.callService(request, (result) => {
        if (!result) {
          this.whiteLEDS_active = !this.whiteLEDS_active;
          alert("Toggling white LEDs failed.");
        }
      });
    },

    toggleCarouselUV: function () {
      this.UV_carousel = !this.UV_carousel;
      let uvService = new ROSLIB.Service({
        ros: this.$ros,
        name: "enable_mosfet_device",
        serviceType: "mrover/EnableDevice",
      });
      let request = new ROSLIB.ServiceRequest({
        name: "uv_led_carousel",
        enable: this.UV_carousel,
      });
      uvService.callService(request, (result) => {
        if (!result) {
          this.UV_carousel = !this.UV_carousel;
          alert("Toggling Carousel UV failed.");
        }
      });
    },
  },
};
</script>

<style scoped>
.box1 {
  text-align: left;
  vertical-align: top;
  display: inline-block;
}

.wrap-table {
  display: inline-block;
  align-content: center;
  height: max-content;
  padding-bottom: 5px;
}

.report {
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
}
</style>
