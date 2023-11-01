import autonomy from "./modules/autonomy";
import erd from "./modules/erd";
import map from "./modules/map";
import { createStore } from 'vuex';

export const store = createStore({
    modules: {
        autonomy,
        erd,
        map,
    }
  })