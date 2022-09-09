import Vue from 'vue'
import Router from 'vue-router'
import MainMenu from '../components/MainMenu.vue'
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
      name: 'MainMenu',
      component: MainMenu
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
    }
  ]
})
