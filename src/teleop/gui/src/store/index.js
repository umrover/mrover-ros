import Vue from 'vue'
import Vuex from 'vuex'
import autonomy from './modules/autonomy'
import sa from './modules/sa'
import erd from './modules/erd'
import map from './modules/map'

Vue.use(Vuex)

const debug = process.env.NODE_ENV !== 'production'

export default new Vuex.Store({
  modules: {
    autonomy,
    sa,
    erd,
    map
  },

  strict: debug
})
