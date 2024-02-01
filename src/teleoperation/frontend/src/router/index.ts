import { createWebHistory, createRouter } from 'vue-router'
import Menu from '../components/Menu.vue'
import DMESTask from '../components/DMESTask.vue'
import AutonTask from '../components/AutonTask.vue'
import ISHTask from '../components/ISHTask.vue'
import SATask from '../components/SATask.vue'

const routes = [
  {
    path: '/',
    name: 'Menu',
    component: Menu
  },
  {
    path: '/DMTask',
    name: 'DMTask',
    component: DMESTask,
    props: {
      type: 'DM'
    }
  },
  {
    path: '/ESTask',
    name: 'ESTask',
    component: DMESTask,
    props: {
      type: 'ES'
    }
  },
  {
    path: '/AutonTask',
    name: 'AutonTask',
    component: AutonTask
  },
  {
    path: '/SATask',
    name: 'SATask',
    component: SATask
  },
  {
    path: '/ISHTask',
    name: 'ISHTask',
    component: ISHTask
  }
]
const router = createRouter({
  history: createWebHistory(),
  routes
})

export default router
