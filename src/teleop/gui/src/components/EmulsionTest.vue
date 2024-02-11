<template>
  <div>
    <h3>Emulsion Testing</h3>
    <button
    id="emulsion-button"
      class="button"
      :disabled="!isEnabled[siteIndex]"
      @click="moveServo(angles[site].pushed, false)"
    >
      Start Site {{ site }} Test
    </button>
    <button id="reset-button" class="button" @click="moveServo(angles[site].start, true)">
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
      servoIDsBySite: {
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

    let servo_id_by_site_config = new ROSLIB.Param({
      ros: this.$ros,
      name: "science/servo_id_by_site",
    });

    let servo_positions = new ROSLIB.Param({
      ros: this.$ros,
      name: "science/servo_positions",
    });

    //get servo angles from yaml file
    servo_id_by_site_config.get((servoIDsBySite) => {
      this.servoIDsBySite = servoIDsBySite;
    });

    for (let site in this.servoIDsBySite) {
      // Get the pushed and start values for the servo from the YAML file
      servo_positions.get((servoPositions) => {
          let pushedVal = servoPositions[this.servoIDsBySite[site]].pushed;
          let startVal = servoPositions[this.servoIDsBySite[site]].start;

          // Add the servo data to the object
          this.angles[site] = {
              pushed: pushedVal,
              start: startVal
          };
      });
    }

  },

  methods: {
    moveServo(angle, enable) {
      let request = new ROSLIB.ServiceRequest({
        id: this.servoIDsBySite[this.site],
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

<style scoped>
.button {
  height: 30px;
  width: 150px;
  border: 1px solid black;
  border-radius: 5px;
  cursor: pointer;
}

.button:hover{
  background-color: rgb(210, 210, 210);
}
</style>
