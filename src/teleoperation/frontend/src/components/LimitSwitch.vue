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
      switch_name: {
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
          if(!msg.result) {
            this.limit_enabled = false;
            alert("Toggling Limit Switch failed.");
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
          name: this.switch_name, 
          enable: this.limit_enabled
        }));
      },
    },
  });
  </script>