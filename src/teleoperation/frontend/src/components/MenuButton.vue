<template>
  <button class="btn btn-primary">
    <a :href="link">{{ name }}</a>
  </button>
</template>
  
<script lang="ts">
import { defineComponent } from 'vue'

export default defineComponent({
  name: "MenuButton",

  props: {
    link: {
      type: String,
      required: true,
    },
    name: {
      type: String,
      required: true,
    },
  },
})
</script>

<style scoped>
  button {
    margin: 5px;
    width: 250px;
    height: 150px;
  }

  a {
    text-decoration: none;
    color: white;
  }
</style>
