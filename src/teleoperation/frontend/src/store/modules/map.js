// Store for map related data
// initial state
const state = {
  odomFormat: 'DM'
}

// getters
const getters = {
  odomFormat: (state) => state.odomFormat
}

// mutations
const mutations = {
  setOdomFormat(state, newOdomFormat) {
    state.odomFormat = newOdomFormat
  }
}

export default {
  namespaced: true,
  state,
  getters,
  mutations
}
