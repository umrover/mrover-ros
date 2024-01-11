<template>
    <input @keydown="keyMonitorDown" />
    <input @keyup="keyMonitorUp" />
</template>
  
  <script lang="ts">
  import { mapActions } from 'vuex';

  const UPDATE_RATE_S = 1;
  let interval:number;
  
  export default {
    data() {
      return {
        rotation_pwr: 0,
        up_down_pwr: 0,
  
        inputData: {
          w_key: 0,
          a_key: 0,
          s_key: 0,
          d_key: 0
        }
      };
    },
  
    beforeUnmount: function () {
      window.clearInterval(interval);
      document.removeEventListener("keyup", this.keyMonitorUp);
      document.removeEventListener("keydown", this.keyMonitorDown);
    },
  
    created: function () {
 
      // Add key listeners.
      document.addEventListener("keyup", this.keyMonitorUp);
      document.addEventListener("keydown", this.keyMonitorDown);
  
      // Publish periodically in case a topic message is missed.
      interval = window.setInterval(() => {
        this.publish();
      }, UPDATE_RATE_S * 1000);
    },
  
    methods: {
      ...mapActions('websocket', ['sendMessage']),
      // When a key is being pressed down, set the power level.
      // Ignore keys that are already pressed to avoid spamming when holding values.
      keyMonitorDown: function (event: { key: string; }) {
        if (event.key.toLowerCase() == "w") {
          if (this.inputData.w_key > 0) {
            return;
          }
          this.inputData.w_key = 1;
        } else if (event.key.toLowerCase() == "a") {
          if (this.inputData.a_key > 0) {
            return;
          }
          this.inputData.a_key = 1;
        } else if (event.key.toLowerCase() == "s") {
          if (this.inputData.s_key > 0) {
            return;
          }
          this.inputData.s_key = 1;
        } else if (event.key.toLowerCase() == "d") {
          if (this.inputData.d_key > 0) {
            return;
          }
          this.inputData.d_key = 1;
        }
  
        this.publish();
      },
  
      // when a key is released, sets input for that key as 0
      keyMonitorUp: function (event: { key: string; }) {
        if (event.key.toLowerCase() == "w") {
          this.inputData.w_key = 0;
        } else if (event.key.toLowerCase() == "a") {
          this.inputData.a_key = 0;
        } else if (event.key.toLowerCase() == "s") {
          this.inputData.s_key = 0;
        } else if (event.key.toLowerCase() == "d") {
          this.inputData.d_key = 0;
        }
  
        this.publish();
      },
  
      publish: function () {
        this.sendMessage({
            type: "mast_gimbal",
            throttles: [this.inputData.d_key - this.inputData.a_key,
                        this.inputData.w_key - this.inputData.s_key]
        });
      }
    }
  };
  </script>
  
  <style scoped>
  </style>