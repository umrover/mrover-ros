<template>
  <a class="btn btn-primary" :href="link" :target="calcTarget">{{ name }}</a>
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

  computed: {
    calcTarget() {
      if (this.name == "Cameras") return ")blank";
      else return "_self"
    }
  }
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
