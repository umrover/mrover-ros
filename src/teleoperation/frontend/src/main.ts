import { createApp } from 'vue'
import App from './App.vue'
import router from './router'

import './app.scss' //custom CSS override file

createApp(App).use(router).mount('#app')