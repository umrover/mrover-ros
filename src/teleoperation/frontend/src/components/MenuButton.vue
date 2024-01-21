<template>
  <a class="btn btn-primary" :href="link">{{ name }}</a>
</template>
<<<<<<< HEAD
  
=======

>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
<script lang="ts">
import { defineComponent } from 'vue'

export default defineComponent({
<<<<<<< HEAD
  name: "MenuButton",
=======
  name: 'MenuButton',
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68

  props: {
    link: {
      type: String,
<<<<<<< HEAD
      required: true,
    },
    name: {
      type: String,
      required: true,
    },
  },
=======
      required: true
    },
    name: {
      type: String,
      required: true
    }
  }
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
})
</script>

<style scoped>
<<<<<<< HEAD
  a {
    margin: 5px;
    width: 250px;
    height: 150px;

    text-decoration: none;
    color: white;
  }
=======
a {
  margin: 5px;
  width: 250px;
  height: 150px;

  text-decoration: none;
  color: white;
}
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
</style>
