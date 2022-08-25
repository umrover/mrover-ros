import Vue from 'vue'
import Vuex from 'vuex'
import autonomy from './modules/autonomy'
import controls from './modules/controls'
import erd from './modules/erd'

Vue.use(Vuex)

const debug = process.env.NODE_ENV !== 'production'

export default new Vuex.Store({
  modules: {
    autonomy,
    controls,
    erd
  },

  strict: debug
})
