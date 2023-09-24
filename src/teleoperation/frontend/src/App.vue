<script setup lang='ts'>
import { RouterLink, RouterView } from 'vue-router'
import TaskSelector from './components/TaskSelector.vue'
</script>

<template>
  <header>
    <div class='wrapper'>
      <TaskSelector />

      <nav>
        <RouterLink to='/autonomy'>Autonomy</RouterLink>
      </nav>
    </div>
  </header>

  <RouterView />
</template>

<style scoped>
</style>
