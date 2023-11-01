import { createWebHistory, createRouter } from "vue-router";
import Menu from "../components/Menu.vue";
import ERDTask from "../components/ERDTask.vue";
import AutonTask from "../components/AutonTask.vue";

const routes = [
  {
    path: "/",
    name: "Menu",
    component: Menu,
  },
  {
    path: "/DMTask",
    name: "DMTask",
    component: ERDTask,
    props: {
      type: "DM",
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
    path: "/AutonTask",
    name: "AutonTask",
    component: AutonTask,
  },
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

export default router;
