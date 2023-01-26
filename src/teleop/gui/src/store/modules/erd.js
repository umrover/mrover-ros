// initial state
const state = {
    waypointList: [],
    highlightedWaypoint: -1,
    odomFormat: "DM",
    clickPoint: {
      lat: 0,
      lon: 0
    },
  
  }
  
  // getters
  const getters = {
    waypointList: state => state.waypointList,
    highlightedWaypoint: state => state.highlightedWaypoint,
    autonEnabled: state => state.autonEnabled,
    teleopEnabled: state => state.teleopEnabled,
    odomFormat: state => state.odomFormat,
    clickPoint: state => state.clickPoint,
  }
  
  // mutations
  const mutations = {
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
  