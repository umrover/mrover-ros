// initial state
const state = {
  camsEnabled: new Array(9).fill(false),
  names: Array.from({ length: 9 }, (_, i) => 'Camera: ' + i),
  cameraIdx: 1,
  cameraName: '',
  capacity: 2,
  qualities: new Array(9).fill(-1),
  streamOrder: [-1, -1, -1, -1]
}

// getters
const getters = {
  camsEnabled: (state) => state.camsEnabled,
  names: (state) => state.names,
  cameraIdx: (state) => state.cameraIdx,
  cameraName: (state) => state.cameraName,
  capacity: (state) => state.capacity,
  qualities: (state) => state.qualities,
  streamOrder: (state) => state.streamOrder
}

// mutations
const mutations = {
  setCamsEnabled(commit, newCamsEnabled) {
    state.camsEnabled = newCamsEnabled
  },

  setNames(commit, newNames) {
    state.names = newNames
  },

  setCameraIdx(commit, newCameraIdx) {
    state.cameraIdx = newCameraIdx
  },
  setCameraName(commit, newCameraName) {
    state.cameraName = newCameraName
  },

  setCapacity(commit, newCapacity) {
    state.capacity = newCapacity
  },

  setQualities(commit, newQualities) {
    state.qualities = newQualities
  },

  setStreamOrder(commit, newStreamOrder) {
    state.streamOrder = newStreamOrder
  }
}

export default {
  namespaced: true,
  state,
  getters,
  mutations
}
