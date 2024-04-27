<template>
  <a class="btn btn-primary" :href="link">{{ name }}</a>
</template>
<script lang="ts">
import { defineComponent } from 'vue'

export default defineComponent({
  name: 'MenuButton',

  props: {
    link: {
      type: String,
      required: true
    },
    name: {
      type: String,
      required: true
    }
  },
})
</script>

<style scoped>
a {
  margin: 5px;
  width: 250px;
  height: 150px;

  text-decoration: none;
  color: white;
}
</style>
