import { createWebHistory, createRouter } from "vue-router";
import Menu from "../components/Menu.vue";

const routes = [
  {
    path: "/",
    name: "Menu",
    component: Menu,
  },
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

export default router;
