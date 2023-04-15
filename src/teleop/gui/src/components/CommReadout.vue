<template>
  <span
    >TX: {{ parseFloat(tx).toFixed(2) }} RX:
    {{ parseFloat(rx).toFixed(2) }}</span
  >
</template>

<script>
import ROSLIB from "roslib/src/RosLib";

export default {
  data() {
    return {
      comm_sub: null,
      tx: 0.0,
      rx: 0.0,
    };
  },

  created() {
    this.comm_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/network_bandwidth",
      messageType: "mrover/NetworkBandwidth",
    });

    this.comm_sub.subscribe((msg) => {
      this.tx = msg.tx;
      this.rx = msg.rx;
    });
  },
};
</script>
