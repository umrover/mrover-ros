<template>
  <div class="menu-btn">
    <a :href="link">{{ name }}</a>
  </div>
</template>

<script>
export default {
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
  }
}
</script>

<style scoped>
.menu-btn {
  font-size: 3em;
  border: black solid 1px;
  border-radius: 5px;
  margin: 0px 5px;
  padding: 0px 5px;
  transition: 0.25s;
  background-color: silver;
}

.menu-btn:hover {
  background-color: gray;
}

.menu-btn a {
  text-decoration: none;
  color: black;
}
</style>
