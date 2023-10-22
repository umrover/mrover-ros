import { createWebHistory, createRouter } from "vue-router";
import Menu from "../components/Menu.vue";
import ERDTask from "../components/ERDTask.vue";

const routes = [
  {
    path: "/",
    name: "Menu",
    component: Menu,
  },
  {
    path: "/EDMTask",
    name: "EDMTask",
    component: ERDTask,
  },
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

export default router;