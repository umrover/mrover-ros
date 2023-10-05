import { createRouter, createWebHistory } from 'vue-router'
import backendView from '../views/backendView.vue'

const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes: [
    {
      path: '/backend',
      name: 'backend',
      component: backendView
    }
  ]
})

export default router
