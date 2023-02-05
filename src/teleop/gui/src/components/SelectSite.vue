<template>
  <div>
    <div>
      <h3>Select Site</h3>
    </div>
    <div class="box1">
      <div>
        <label for="position">Change site to position:</label>
        <select id="site" v-model="site" name="site" @change="onChange($event)">
          <option value="A">Position A</option>
          <option value="B">Position B</option>
          <option value="C">Position C</option>
        </select>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  components: {},
  data() {
    return {
      site: "A",
    };
  },

  methods: {
    onChange(event) {
      this.$emit("site", event.target.value);
    },
  },
};
</script>

<style scoped>
.box1 {
  text-align: left;
  vertical-align: top;
  display: inline-block;
}
</style>
