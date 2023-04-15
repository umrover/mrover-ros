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
  display: flex;
  width:100%;
  height:100%;
}
.spacer:hover {
  background-color: var(--shadow-color);
}

a {
  text-decoration: none;
  text-align: center;
  width: 100%;
  height: 100%;
  color: var(--primary-blue);
}
</style>
