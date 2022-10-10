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
  font-family: 'Avenir', Helvetica, Arial, sans-serif, "Product Sans";
}

@font-face {
  font-family: "Product Sans";
  src: url('/assets/fonts/Product Sans Regular.ttf');
  font-weight: normal;
  font-style: normal;
}

@font-face {
  font-family: "Product Sans";
  src: url('/assets/fonts/Product Sans Bold.ttf');
  font-weight: bold;
  font-style: normal;
}

@font-face {
  font-family: "Product Sans";
  src: url('/assets/fonts/Product Sans Bold Italic.ttf');
  font-weight: bold;
  font-style: italic;
}

@font-face {
  font-family: "Product Sans";
  src: url('/assets/fonts/Product Sans Italic.ttf');
  font-weight: normal;
  font-style: italic;
}
</style>
