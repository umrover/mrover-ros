import { createApp } from 'vue'
import App from './App.vue'
import router from './router'
import { store } from './store'
import VueApexCharts from "vue3-apexcharts";

import './app.scss' //custom CSS override file

const app: App<Element> = createApp(App)
app.use(router).use(store).use(VueApexCharts).mount('#app')
store.dispatch('websocket/setupWebSocket')
