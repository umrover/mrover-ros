<template>
  <div class="wrap">
    <div>
      <h4>Adjust Joint Angles</h4>
      <div>
        <label for="joint">Joint to adjust</label>
        <select v-model="selectedJoint">
          <option disabled value="">Select a joint</option>
          <option value="joint_a">A</option>
          <option value="joint_b">B</option>
          <option value="joint_c">C</option>
          <option value="joint_d">D</option>
          <option value="joint_e">E</option>
        </select>
        <label for="angle">Adjustment Angle</label>
        <input
          v-model="adjustmentAngle"
          type="number"
          :min="-2 * Math.PI"
          :max="2 * Math.PI"
        />
        <input type="button" value="Adjust" @click="publishAdjustmentMessage" />
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
export default {
  data() {
    return {
      adjustmentAngle: 0,
      selectedJoint: "",

      serviceClient: null
    };
  },

  created() {
    this.serviceClient = new ROSLIB.Service({
      ros: this.$ros,
      name: "/adjust",
      serviceType: "mrover/AdjustMotors"
    });
  },

  methods: {
    publishAdjustmentMessage() {
      const request = new ROSLIB.ServiceRequest({
        name: this.selectedJoint,
        value: this.clamp(parseFloat(this.adjustmentAngle), -2 * Math.PI, 2 * Math.PI)
      });
      if (this.selectedJoint != "") {
        this.serviceClient.callService(request, (result) => {
			if(!result.success){
				alert("Adjustment failed");
			}
		});
      }
    },

	clamp(value, min, max) {
	  return Math.min(Math.max(value, min), max);
	}
  }
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
