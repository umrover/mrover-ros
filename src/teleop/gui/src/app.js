// The Vue build version to load with the `import` command
// (runtime-only or standalone) has been set in webpack.base.conf with an alias.
import Vue from "vue";
import App from "./App.vue";
import ROSLIB from "roslib";
import router from "./router";
import store from "./store";
import "leaflet/dist/leaflet.css";

Vue.config.productionTip = false;
Vue.prototype.$ros = new ROSLIB.Ros({
  url: "ws://10.0.0.7:9090",
});

// For whether we are on basestation or not
Vue.prototype.$competitionMode = false;

/* eslint-disable no-new */
new Vue({
  el: "#app",
  router,
  store,
  components: { App },
  template: "<App/>",
});
