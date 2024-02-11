<template>
  <div class="wrapper">
    <div class="box header">
      <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
      <h1>Mrover Teleop Training</h1>
      <div class="spacer"></div>
    </div>
  </div>
</template>

<script>

export default {
  name: 'Menu',

}
</script>

<style scoped>
.box {
  border-radius: 5px;
  padding: 10px;
  border: 1px solid black;
}

.header {
  grid-area: header;
  display: flex;
  align-items: center;
}

.header h1 {
  margin-left: 5px;
}

img {
  border: none;
  border-radius: 0px;
}

.pages {
  grid-area: pages;
  border: black solid 1px;
  border-radius: 5px;
  background-color: lightgray;
}

.row {
  display: flex;
  border: black solid 1px;
  border-radius: 5px;
  background-color: darkgray;
  margin: 5px;
}

.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: 1fr;
  grid-template-rows: 60px 1fr;
  grid-template-areas: "header" "pages";
  font-family: sans-serif;
  height: auto;
}

</style>
