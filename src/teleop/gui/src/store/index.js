import Vue from 'vue'
import Vuex from 'vuex'
import autonomy from './modules/autonomy'

Vue.use(Vuex)

const debug = process.env.NODE_ENV !== 'production'

export default new Vuex.Store({
  modules: {
    autonomy,
  },

  strict: debug
})
