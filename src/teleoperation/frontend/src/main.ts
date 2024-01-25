import { createApp } from 'vue'
import App from './App.vue'
import router from './router'
import { store } from './store'

import './app.scss' //custom CSS override file

const app: App<Element> = createApp(App)
app.use(router).use(store).mount('#app')
store.dispatch('websocket/setupWebSocket')
