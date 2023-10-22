import { createRouter, createWebHistory } from 'vue-router'
import AutonomyView from '../views/AutonomyView.vue'

const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes: [
    {
      path: '/autonomy',
      name: 'autonomy',
      component: AutonomyView
    }
  ]
})

export default router
