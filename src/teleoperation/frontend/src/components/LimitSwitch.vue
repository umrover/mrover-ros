<template>
    <div class="wrap">
      <ToggleButton
        :id="name"
        :current-state="limit_enabled"
        :label-enable-text="name + ' On'"
        :label-disable-text="name + ' Off'"
        @change="toggleLimitSwitch()"
      />
    </div>
  </template>
  
  <script lang="ts">
  import  { defineComponent, inject } from "vue";
  import ToggleButton from "./ToggleButton.vue";
  
  export default defineComponent({
    components: {
      ToggleButton,
    },
    props: {
      name: {
        type: String,
        required: true,
      },
    },
  
    data() {
      return {
        websocket: inject("webSocketService") as WebSocket,
        limit_enabled: false,
      };
    },
  
    created: function () {
      this.websocket.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        if(msg.type == "enable_device_srv") {
          if(msg.result.length>0) {
            this.limit_enabled = false;
            for (var j = 0; j<msg.result.length;++j)
              {
                alert("Toggling Limit Switch failed for" + msg.result[j]);
              }
          }
        }
      };

      this.toggleLimitSwitch(); //enables the limit switch when created
    },
  
    methods: {
      toggleLimitSwitch: function () {
        this.limit_enabled = !this.limit_enabled;
        this.websocket.send(JSON.stringify({
          type: "enable_device_srv", 
          data: this.limit_enabled
        }));
      },
    },
  });
  </script>