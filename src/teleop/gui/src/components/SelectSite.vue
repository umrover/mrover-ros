<template>
    <div>
      <div>
          <h3> Select Site </h3>
      </div>
      <div class="box1">
        <div>
          <label for="position">Change site to position:</label>
          <select>
            <option> Position A </option>
            <option> Position B </option>
            <option> Position C </option>
          </select>
        </div>
      </div>
    </div>
</template>

<script>

export default {
  data() {
    return {

    }
  },

  components:{
  }
}
</script>

<style scoped>
  .box1 {
    text-align: left;
    vertical-align: top;
    display: inline-block;
  }
</style>