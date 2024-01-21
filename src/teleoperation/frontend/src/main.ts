import { createApp } from 'vue'
import App from './App.vue'
import router from './router'
import { store } from './store'

import './app.scss' //custom CSS override file

// const ws:WebSocket = new WebSocket('ws://localhost:8000/ws/gui');

// ws.onopen = () => {
// 	  console.log('WebSocket connection opened successfully.');
<<<<<<< HEAD
	  const app:App<Element> = createApp(App);
	//   app.provide('webSocketService', ws);
	  app.use(router).use(store).mount('#app');
	  store.dispatch('websocket/setupWebSocket');
=======
const app: App<Element> = createApp(App)
//   app.provide('webSocketService', ws);
app.use(router).use(store).mount('#app')
store.dispatch('websocket/setupWebSocket')
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
// }
