import Vue from 'vue'
import Vuex from 'vuex'
import autonomy from './modules/autonomy'
import sa from './modules/sa'

Vue.use(Vuex)

const debug = process.env.NODE_ENV !== 'production'

export default new Vuex.Store({
  modules: {
    autonomy,
    sa
  },

  strict: debug
})
