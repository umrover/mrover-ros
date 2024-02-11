// The Vue build version to load with the `import` command
// (runtime-only or standalone) has been set in webpack.base.conf with an alias.
import Vue from 'vue'
import App from './App.vue'
import ROSLIB from "roslib"
import router from './router'
import 'leaflet/dist/leaflet.css'

Vue.config.productionTip = false
Vue.prototype.$ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'
});

/* eslint-disable no-new */
new Vue({
  el: '#app',
  router,
  components: { App },
  template: '<App/>'
})
