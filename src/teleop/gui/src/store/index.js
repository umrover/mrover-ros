import Vuex from "vuex";
import autonomy from "./modules/autonomy";
import erd from "./modules/erd";
import map from "./modules/map";

const debug = process.env.NODE_ENV !== "production";

export default Vuex.createStore({
  modules: {
    autonomy,
    erd,
    map,
  },

  strict: debug,
});
