<template>
  <div>
    <h3>Emulsion Testing</h3>
    <button
      id="sudan-button"
      :disabled="!isEnabled[siteIndex]"
      @click="moveServo(angles[site].pushed, false)"
    >
      Start Site {{ site }} Test
    </button>
    <button id="reset-button" @click="moveServo(angles[site].start, true)">
      Reset Site {{ site }} Servo
    </button>
  </div>
</template>

<script>
import ROSLIB from "roslib/src/RosLib";
import Vue from "vue";

export default {
  props: {
    site: {
      type: String,
      required: true,
    },
    siteIndex: {
      type: Number,
      required: true,
    },
  },
  data() {
    return {
      isEnabled: [true, true, true],
      angles: {
        A: {
          pushed: null,
          start: null,
        },
        B: {
          pushed: null,
          start: null,
        },
        C: {
          pushed: null,
          start: null,
        },
      },
      servoClient: null,
    };
  },

  created: function () {
    this.serviceClient = new ROSLIB.Service({
      ros: this.$ros,
      name: "change_servo_angles",
      serviceType: "mrover/ChangeServoAngle",
    });

    let param = new ROSLIB.Param({
      ros: this.$ros,
      name: "science/syringe_servo_positions",
    });
    //get servo angles from yaml file
    param.get((angles) => {
      let letter = "A";
      for (var i = 0; i < 3; i++) {
        this.angles[letter].pushed = angles["site_" + letter]["pushed"];
        this.angles[letter].start = angles["site_" + letter]["start"];
        letter = String.fromCharCode(letter.charCodeAt(0) + 1);
      }
    });
  },

  methods: {
    moveServo(angle, enable) {
      let request = new ROSLIB.ServiceRequest({
        id: this.siteIndex,
        angle: angle,
      });

      this.serviceClient.callService(request, (result) => {
        if (!result.success)
          alert(
            "Changing servo angle at site " + this.site + " was not successful"
          );
        else Vue.set(this.isEnabled, this.siteIndex, enable);
      });
    },
  },
};
</script>

<style scoped></style>
