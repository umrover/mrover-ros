import { createWebHistory, createRouter } from "vue-router";
import Menu from "../components/Menu.vue";
import ControlGui from "../components/controlgui.vue"
const routes = [
  {
    path: "/",
    name: "Menu",
    component: Menu,
  },
  {
    path: "/",
    name: "Control Gui",
    component: ControlGui
  }
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

export default router;
