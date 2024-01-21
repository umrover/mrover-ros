// initial state
const state = {
  route: [],
  waypointList: [],
  highlightedWaypoint: -1,
  autonEnabled: false,
  teleopEnabled: false,
  odomFormat: 'DM',
  clickPoint: {
    lat: 0,
    lon: 0
  }
}

// getters
const getters = {
  route: (state) => state.route,
  waypointList: (state) => state.waypointList,
  highlightedWaypoint: (state) => state.highlightedWaypoint,
  autonEnabled: (state) => state.autonEnabled,
  teleopEnabled: (state) => state.teleopEnabled,
  odomFormat: (state) => state.odomFormat,
  clickPoint: (state) => state.clickPoint
}

// mutations
const mutations = {
  setRoute(state, newRoute) {
    state.route = newRoute
  },

  setAutonMode(state, newAutonEnabled) {
    state.autonEnabled = newAutonEnabled
  },

  setTeleopMode(state, newTeleopEnabled) {
    state.teleopEnabled = newTeleopEnabled
  },

  setWaypointList(state, newList) {
    state.waypointList = newList
  },

  setHighlightedWaypoint(state, newWaypoint) {
    state.highlightedWaypoint = newWaypoint
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
