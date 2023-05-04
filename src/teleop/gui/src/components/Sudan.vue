<template>
  <div>
    <h3>Sudan III Drop</h3>
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
      servo_id_by_site: {
        A: 0,
        B: 1,
        C: 2,
      },
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
      name: "change_servo_angle",
      serviceType: "mrover/ChangeServoAngle",
    });

    let servo_id_by_site = new ROSLIB.Param({
      ros: this.$ros,
      name: "science/servo_id_by_site",
    });

    let servo_positions = new ROSLIB.Param({
      ros: this.$ros,
      name: "science/servo_positions",
    });

    //get servo angles from yaml file
    servo_id_by_site.get((servoIDsBySite) => {
      for (let site in servoIDsBySite) {
        let servoNum = servoIDsBySite[site];

        this.servo_id_by_site[site] = servoNum;

        // Get the pushed and start values for the servo from the YAML file
        servo_positions.get((servoPositions) => {
            let pushedVal = servoPositions[`servo_${servoNum}`].pushed;
            let startVal = servoPositions[`servo_${servoNum}`].start;

            // Add the servo data to the object
            this.angles[site] = {
                pushed: pushedVal,
                start: startVal
            };
        });
      }
    });
  },

  methods: {
    moveServo(angle, enable) {
      let request = new ROSLIB.ServiceRequest({
        id: this.servo_id_by_site[this.site],
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
