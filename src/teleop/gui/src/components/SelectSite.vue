<template>
  <div>
    <div>
      <h3>Select Site</h3>
    </div>
    <div class="box1">
      <div>
        <label for="position">Change site to position:</label>
        <input type="radio" id="site" v-model="site" name="site" value="A" @change="onChange($event)"> A
        <input type="radio" id="site" v-model="site" name="site" value="B" @change="onChange($event)"> B
        <input type="radio" id="site" v-model="site" name="site" value="C" @change="onChange($event)"> C
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
