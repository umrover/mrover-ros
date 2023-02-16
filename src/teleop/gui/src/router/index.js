import Vue from "vue";
import Router from "vue-router";
import Menu from "../components/Menu.vue";
import ERDTask from "../components/ERDTask.vue";
import AutonTask from "../components/AutonTask.vue";
import SATask from "../components/SATask.vue";
import ISHTask from "../components/ISHTask.vue";
import ESWDebugGUI from "../components/ESWDebugGUI.vue";
import ROSDebug from "../components/ROSDebug.vue";
import ROSSend from "../components/ROSSend.vue";
import ROSEcho from "../components/ROSEcho.vue";
import ROSService from "../components/ROSService.vue";

Vue.use(Router);

export default new Router({
  routes: [
    {
      path: "/",
      name: "Menu",
      component: Menu,
    },
    {
      path: "/EDMTask",
      name: "EDMTask",
      component: ERDTask,
      props: {
        type: "EDM",
      },
    },
    {
      path: "/ESTask",
      name: "ESTask",
      component: ERDTask,
      props: {
        type: "ES",
      },
    },
    {
      path: "/SATask",
      name: "SATask",
      component: SATask,
    },
    {
      path: "/ISHTask",
      name: "ISHTask",
      component: ISHTask,
    },
    {
      path: "/AutonTask",
      name: "AutonTask",
      component: AutonTask,
    },
    {
      path: "/ESWDebug",
      name: "ESWDebugGUI",
      component: ESWDebugGUI,
    },
    {
      path: "/ROSDebug",
      name: "ROSDebug",
      component: ROSDebug,
    },
    {
      path: "/ROSSend",
      name: "ROSSend",
      component: ROSSend,
    },
    {
      path: "/ROSEcho",
      name: "ROSEcho",
      component: ROSEcho,
    },
    {
      path: "/ROSService",
      name: "ROSService",
      component: ROSService,
    },
  ],
});
