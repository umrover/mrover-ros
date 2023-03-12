<template>
    <div class="wrap">
      <span>TX: {{tx}} RX: {{rx}}</span>
    </div>
  </template>
  
  <script>
import ROSLIB from 'roslib/src/RosLib';

  export default {
    data() {
        return {
            comm_sub: null,
            tx: "0.00",
            rx: "0.00"
        };
    },

    created() {
        this.comm_sub = new ROSLIB.Topic({
            ros: this.$ros,
            name: "/network_bandwidth",
            messageType: "mrover/NetworkBandwidth"
        });

        this.comm_sub.subscribe((msg) => {
            this.tx = msg.tx.toFixed(2);  //prints out to 2 decimals
            this.rx = msg.rx.toFixed(2);
        });
    }
  }
  </script>
  
  <style scoped>
    .wrap {
      display: flex;
      align-items: center;
      height: 100%;
    }
  </style>