// Import any necessary WebSocket library or use the native WebSocket API
const webSocket = new WebSocket('ws://localhost:8000/ws/gui')

const state = {
  message: null
}

const mutations = {
  setMessage(state, payload) {
    state.message = payload
  }
}

const actions = {
  sendMessage({ commit }, message) {
    // Send message through the WebSocket
    webSocket.send(JSON.stringify(message))
  },

  setupWebSocket({ commit, dispatch }) {
    // Set up WebSocket event listeners
    webSocket.onmessage = (event) => {
      const message = JSON.parse(event.data)
      commit('setMessage', message)
      // Optionally, dispatch other actions based on the received message
    }

    // Add other event listeners as needed (onopen, onclose, onerror, etc.)
  }
}

export default {
  namespaced: true,
  state,
  mutations,
  actions
}
