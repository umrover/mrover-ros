const state = {
  controlMode: ''
}

// getters
const getters = {
  controlMode: state => state.controlMode
}

// mutations
const mutations = {
  setControlMode (commit, newMode) {
    state.controlMode = newMode
  },
}

export default {
  namespaced: true,
  state,
  getters,
  mutations
}
