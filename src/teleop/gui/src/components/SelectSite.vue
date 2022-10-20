<template>
    <div>
      <div>
          <h3> Select Site </h3>
      </div>
      <div class="box1">
        <div>
          <label for="position">Change site to position:</label>
          <select>
            <option> Position 0 </option>
            <option> Position 1 </option>
            <option> Position 2 </option>
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