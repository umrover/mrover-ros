<template>
  <div class="spacer">
    <a :href="link">
      <img :src="require(`../assets/${name}.svg`).default"/>
      <p>{{ name }}</p>
    </a>
  </div>
</template>

<script>
import "../assets/style.css";

export default {
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
  }

};
</script>

<style scoped>

.spacer {
  margin: 10px;
}

a {
  text-decoration: none;
  color: var(--primary-blue);
}
</style>
