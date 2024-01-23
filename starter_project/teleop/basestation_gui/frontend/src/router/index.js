import { createWebHistory, createRouter } from "vue-router";
import Menu from "../components/MenuPage.vue";
import DriveControls from "../components/DriveControls.vue";

const routes = [
  {
    path: "/",
    name: "Menu",
    component: Menu,
  },
  {
    path: '/motor_sim',
    name: 'Motor Sim',
    component: DriveControls
  },
];

const router = createRouter({
  history: createWebHistory(),
  routes,
});

export default router;
