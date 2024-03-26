<template>
  <div>
    <h3>Chlorophyll Test Control</h3>
    <div>
      <div>
        <ToggleButton id="white_led_toggle" :current-state="whiteLEDs_active" label-enable-text="White LEDs On"
          label-disable-text="White LEDs Off" @change="toggle_whiteLEDS()" />
      </div>
      <div>
        <ToggleButton id="uv_led_toggle" :current-state="uvLEDs_active" label-enable-text="UV LEDs On"
          label-disable-text="UV LEDs Off" @change="toggle_uvLEDS()" />
      </div>
    </div>
    <div class="wrap-table" id="capture">
      <div>
        <h3>Chlorophyll Spectral data</h3>
      </div>
      <table class="table table-bordered">
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
              Site {{ i-1 }}
            </td>
            <td v-if="!error[i - 1]" v-for="j in 6" :key="j">
              {{ spectral_data[i - 1][j - 1].toFixed(2) }}
            </td>
            <td v-else v-for="k in 6" :key="k">
              -
            </td>
          </tr>
        </tbody>
      </table>
    </div>
    <div>
      <button class="btn btn-primary" @click="download_data(spectral_data)">Generate Report</button>
    </div>
  </div>
</template>

<script lang="ts">
import html2canvas from "html2canvas";
import ToggleButton from "./ToggleButton.vue";
import { mapState, mapActions } from 'vuex'

export default {
  components: {
    ToggleButton,
  },

  data() {
    return {
      whiteLEDs_active: false,
      uvLEDs_active: false,
      spectral_data: [
        [],
        [],
        []
      ],
      error: [true, true, true],
    };
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == 'spectral_data') {
        this.spectral_data[msg.site] = msg.data
        this.error[msg.site] = msg.error
      }
      else if (msg.type == 'enable_white_leds'){
        if(!msg.result){
          this.whiteLEDs_active = false
        }
      }
      // TODO: get white LED message back and fix toggle if need be
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    toggle_whiteLEDS: function () {
      this.whiteLEDs_active = !this.whiteLEDs_active;
      this.sendMessage({ type: 'enable_white_leds', data: this.whiteLEDs_active })
    },

    toggle_uvLEDS: function () {
      this.uvLEDs_active = !this.uvLEDs_active;
      this.sendMessage({ type: 'enable_uv_leds', data: this.uvLEDs_active })
    },

    download_data: function(spectral_data: any) {
      // generates report
      this.sendMessage({type:'download_csv', data:spectral_data})

      // downloads screenshot of table
      const table = document.querySelector("#capture") as HTMLElement;
      html2canvas(table)
      .then(canvas => {
        canvas.style.display = 'none'
        document.body.appendChild(canvas)
        return canvas
      })
      .then(canvas => {
        const image = canvas.toDataURL('image/png')
        const a = document.createElement('a')
        a.setAttribute('download', 'chlorophyll.png')
        a.setAttribute('href', image)
        a.click()
        canvas.remove()
      })
    },
  },

};
</script>

<style scoped>

.wrap-table {
  display: inline-block;
  align-content: center;
  height: max-content;
  padding-bottom: 5px;
}

#btn {
  margin-left: 10px;
}

</style>