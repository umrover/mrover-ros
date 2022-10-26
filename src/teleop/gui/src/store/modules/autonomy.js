// initial state
const state = {
  route: [],
  waypointList: [],
  highlightedWaypoint: -1,
  autonEnabled: false,
  teleopEnabled: true,
  odomFormat: "DM",
  clickPoint: {
    lat: 0,
    lon: 0
  },

}

// getters
const getters = {
  route: state => state.route,
  waypointList: state => state.waypointList,
  highlightedWaypoint: state => state.highlightedWaypoint,
  autonEnabled: state => state.autonEnabled,
  teleopEnabled: state => state.teleopEnabled,
  odomFormat: state => state.odomFormat,
  clickPoint: state => state.clickPoint,
}

// mutations
const mutations = {
  setRoute (commit, newRoute) {
    state.route = newRoute
  },

  setAutonMode (commit, newAutonEnabled) {
    state.autonEnabled = newAutonEnabled
  },

  setTeleopMode (commit, newTeleopEnabled) {
    state.teleopEnabled = newTeleopEnabled
  },

  setWaypointList (commit, newList) {
    state.waypointList = newList
  },

  setHighlightedWaypoint (commit, newWaypoint) {
    state.highlightedWaypoint = newWaypoint
  },

  setOdomFormat (commit, newOdomFormat) {
    state.odomFormat = newOdomFormat
  },

  setClickPoint (commit, newClickPoint) {
    state.clickPoint = newClickPoint
  },
}

export default {
  namespaced: true,
  state,
  getters,
  mutations
}
