<template>
  <div>
    <div>
      <h2>Select Site</h2>
    </div>
    <div class="radio-buttons">
      <label>Change site to:</label>
      <input
        id="site-a"
        v-model="site"
        type="radio"
        name="site"
        value="A"
        :class="{ selected: site === 'A' }"
        @change="onChange($event)"
      />
      <label for="site-a" :class="{ selected: site === 'A' }">A</label>
      <input
        id="site-b"
        v-model="site"
        type="radio"
        name="site"
        value="B"
        :class="{ selected: site === 'B' }"
        @change="onChange($event)"
      />
      <label for="site-b" :class="{ selected: site === 'B' }">B</label>
      <input
        id="site-c"
        v-model="site"
        type="radio"
        name="site"
        value="C"
        :class="{ selected: site === 'C' }"
        @change="onChange($event)"
      />
      <label for="site-c" :class="{ selected: site === 'C' }">C</label>
    </div>
  </div>
</template>

<script>
export default {
  components: {},
  data() {
    return {
      site: "A"
    };
  },

  methods: {
    onChange(event) {
      this.$emit("site", event.target.value);
    }
  }
};
</script>

<style scoped>
.radio-buttons {
  text-align: left;
  vertical-align: top;
  display: inline-block;
  font-size: larger;
}

.selected {
  padding: 3px;
  background-color: #4caf50;
  color: white;
  border: 1px solid black;
  border-radius: 5px;
}

.radio-buttons input {
  -ms-transform: scale(1.5); /* IE 9 */
  -webkit-transform: scale(1.5); /* Chrome, Safari, Opera */
  transform: scale(1.5);
}
</style>
