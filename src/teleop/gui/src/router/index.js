import Vue from 'vue'
import Router from 'vue-router'
import Menu from '../components/Menu.vue'
import ERDTask from '../components/ERDTask.vue'
import ESTask from '../components/ESTask.vue'
import AutonTask from '../components/AutonTask.vue'
import SATask from '../components/SATask.vue'
import ISHTask from '../components/ISHTask.vue'
import ControlGUI from '../components/ControlGUI.vue'

Vue.use(Router)

export default new Router({
  routes: [
    {
      path: '/',
      name: 'Menu',
      component: Menu
    },
    {
      path: '/ERDTask',
      name: 'ERDTask',
      component: ERDTask
    },
    {
      path: '/ESTask',
      name: 'ESTask',
      component: ESTask
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
    },
    {
      path: '/AutonTask',
      name: 'AutonTask',
      component: AutonTask
    },
    {
      path: '/Control',
      name: 'ControlsGUI',
      component: ControlGUI
    },
  ]
})
