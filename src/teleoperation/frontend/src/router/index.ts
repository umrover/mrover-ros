import { createWebHistory, createRouter } from "vue-router";
import Menu from "../components/Menu.vue";
import DMESTask from "../components/DMESTask.vue";
import AutonTask from "../components/AutonTask.vue";
import Rover3D from "../components/Rover3D.vue";

const routes = [
  {
    path: "/",
    name: "Menu",
    component: Menu,
  },
  {
    path: "/DMTask",
    name: "DMTask",
    component: DMESTask,
    props: {
      type: "DM",
    },
  },
  {
    path: "/ESTask",
    name: "ESTask",
    component: DMESTask,
    props: {
      type: "ES",
    },
  },
  {
    path: "/AutonTask",
    name: "AutonTask",
    component: AutonTask,
  },
  {
    path: "/Control",
    name: "Control",
    component: Rover3D,
  },
];
const router = createRouter({
  history: createWebHistory(),
  routes,
});

export default router;