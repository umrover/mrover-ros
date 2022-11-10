<template>
  <div id="app">
    <router-view/>
  </div>
</template>

<script>
import ROSLIB from "roslib"

export default {
  name: 'App',

  beforeCreate: function () {
    this.$ros.on('connection', function() {
        console.log('Connected to websocket server.');
    });

    this.$ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
    });

    this.$ros.on('close', function() {
        console.log('Connection to websocket server closed.');
    });
  }
}
</script>

<style scoped>
#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
}

</style>
