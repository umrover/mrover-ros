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

<style>
#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  height: 100%;
}

* {
  --primary-blue: rgb(63, 99, 166);
  --hover-blue: rgb(79, 119, 192);
  --active-blue: rgb(112, 148, 216);
  --shadow-color: rgba(236, 236, 236, 0.966);
}

</style>
