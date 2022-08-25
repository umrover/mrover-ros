// initial state
const state = {
  waypointList: [],
  odomFormat: "DM",
  highlightedWaypoint: -1,
  clickPoint: {
    lat: 0,
    lon: 0
  },
}

// getters
const getters = {
  waypointList: state => state.waypointList,
  odomFormat: state => state.odomFormat,
  highlightedWaypoint: state => state.highlightedWaypoint,
  clickPoint: state => state.clickPoint
}

// mutations
const mutations = {
  setWaypointList (commit, newList) {
    state.waypointList = newList
  },

  setOdomFormat (commit, newOdomFormat) {
    state.odomFormat = newOdomFormat
  },

  setHighlightedWaypoint (commit, newWaypoint) {
    state.highlightedWaypoint = newWaypoint
  },

  setClickPoint (commit, newClickPoint) {
    state.clickPoint = newClickPoint
  }
}

export default {
  namespaced: true,
  state,
  getters,
  mutations
}
