<template>
  <div class="wrap">
    <div>
      <h4>Adjust Joint Angles</h4>
	  <div>
		  <label for="joint">Joint to adjust</label>
		  <select v-model="selectedJoint">
			<option disabled value="">Please select a joint</option>
			<option>A</option>
			<option>B</option>
			<option>C</option>
			<option>D</option>
			<option>E</option>
		  </select>
		  <label for="angle">Adjustment Angle</label>
		  <input
			v-model="adjustmentAngle"
			type="number"
			:min="0"
			:max="2 * Math.PI"
		  />
	  </div>
    </div>
  </div>
</template>

<script>
export default {
  data() {
    return {
      adjustmentAngle: 0
    };
  },

  created() {},

  methods: {}
};
</script>

<style scoped>
/* make items appear in one row */
.wrap {
  display: flex;
  height: 100%;
}

.wrap h4 {
  margin-top: -10px;
  margin-bottom: 10px;
}

</style>
