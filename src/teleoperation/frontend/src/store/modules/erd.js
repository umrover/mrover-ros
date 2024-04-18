// initial state
const state = {
  waypointList: [],
  highlightedWaypoint: -1,
  searchWaypoint: -1,
  odomFormat: 'DM',
  clickPoint: {
    lat: 0,
    lon: 0
  }
}

// getters
const getters = {
  waypointList: (state) => state.waypointList,
  highlightedWaypoint: (state) => state.highlightedWaypoint,
  searchWaypoint: (state) => state.searchWaypoint,
  odomFormat: (state) => state.odomFormat,
  clickPoint: (state) => state.clickPoint
}

// mutations
const mutations = {
  setWaypointList(state, newList) {
    state.waypointList = newList
  },

  setHighlightedWaypoint(state, newWaypoint) {
    state.highlightedWaypoint = newWaypoint
  },

  setSearchWaypoint(state, newWaypoint) {
    state.searchWaypoint = newWaypoint
  },

  setOdomFormat(state, newOdomFormat) {
    state.odomFormat = newOdomFormat
  },

  setClickPoint(state, newClickPoint) {
    state.clickPoint = newClickPoint
  }
}

export default {
  namespaced: true,
  state,
  getters,
  mutations
}
