<!--
  MotorAdjust.vue

  Component for adjusting the angle of a motor using the AdjustMotors service.
  If multiple motors are passed in, the user can select which motor to adjust.
  Otherwise, the motor to adjust is the only motor passed in.

  Properties:
    options: Array of objects in format {name: "joint_a", option: "Joint A"}
      Required
      Default to empty array
      If array is empty, no motor adjustment UI is shown
      If array has one element, the motor to adjust is the only element
      If array has multiple elements, the user can select which motor to adjust
fit
  Example:
    <MotorAdjust :options="[{name: 'joint_a', option: 'Joint A'}]" />
-->
<template>
    <div class="wrap">
      <div v-if="options.length > 1">
        <h4>Adjust Motor Angles</h4>
      </div>
      <div v-else>
        <h4>Adjust {{ options[0].option }} Angle</h4>
      </div>
      <div>
        <div v-if="options.length > 1">
          <label for="joint">Motor to adjust</label>
          <select v-model="selectedMotor">
            <option disabled value="">Select a motor</option>
            <option
              v-for="option in options"
              :key="option.name"
              :value="option.name"
            >
              {{ option.option }}
            </option>
          </select>
        </div>
        <div>
          <label for="angle">Angle (in Rad)</label>
          <input
            v-model="adjustmentAngle"
            type="number"
            :min="-2 * Math.PI"
            :max="2 * Math.PI"
          />
          <input
            class="submit-button"
            type="button"
            value="Adjust"
            @click="publishAdjustmentMessage"
          />
        </div>
      </div>
    </div>
  </template>
  
  <script lang = "ts">
import { inject, defineComponent } from 'vue';
  export default {
    props: {
      options: {
        type: Array <{ name: string; option: string }>,
        required: true,
        // Default to empty array
        // Should be array of object in format {name: "joint_a", option: "A"}
        default: () => [],
      },
    },
  
    data() {
      return {
        websocket: new WebSocket("ws://localhost:8000/ws/gui"),
        adjustmentAngle: 0,
        selectedMotor: "",
  
        serviceClient: null,
      };
    },
  

    created: function () {
        this.websocket.onmessage = (event) => {
            const msg = JSON.parse(event.data);
            if(msg.type=="arm_adjust"){
              if (!msg.success) {
              alert("Adjustment failed");
            }
            }
        };

        if (this.options.length == 1) {
        this.selectedMotor = this.options[0].name;
      }
    },
  
  
    methods: {
      publishAdjustmentMessage() {
        if (this.selectedMotor != "") {
          this.websocket.send(JSON.stringify({ type: "arm_adjust", name: this.selectedMotor,
          value: this.clamp(
            parseFloat(this.adjustmentAngle.toString()),
            -2 * Math.PI,
            2 * Math.PI
          ),}))
        }
      },
  
      clamp(value: number, min: number, max: number) {
        return Math.min(Math.max(value, min), max);
      },
    },
  };
  </script>
  
  <style scoped>
  /* make items appear in one row */
  .wrap {
    display: flex;
    height: 100%;
    padding: 1.5% 0 1.5% 0;
    flex-direction: column;
  }
  
  .wrap h4 {
    margin-top: -10px;
    margin-bottom: 10px;
  }
  
  .submit-button {
    height: 30px;
    width: 75px;
    border: 1px solid black;
    border-radius: 5px;
    cursor: pointer;
  }
  
  .submit-button:hover {
    background-color: rgb(210, 210, 210);
  }
  </style>