let globalCommit = null

let webSocket = null

function seutpWebsocket() {
  webSocket = new WebSocket('ws://localhost:8000/ws/gui')

  webSocket.onopen = () => {
    console.log('WebSocket Client Connected')
  }
  webSocket.onmessage = event => {
    const message = JSON.parse(event.data)
    if (globalCommit) globalCommit('setMessage', message)
  }
  webSocket.onclose = e => {
    console.log('Socket is closed. Reconnect will be attempted in 1 second.', e.reason)
    setTimeout(() => {
      seutpWebsocket()
    }, 2000)
  }
  webSocket.onerror = error => {
    console.error('Socket encountered error', error)
    webSocket.close()
  }
}

seutpWebsocket()

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
    if (webSocket.readyState !== WebSocket.OPEN) return

    webSocket.send(JSON.stringify(message))
  },

  setupWebSocket({ commit }) {
    globalCommit = commit
  }
}

export default {
  namespaced: true,
  state,
  mutations,
  actions
}
