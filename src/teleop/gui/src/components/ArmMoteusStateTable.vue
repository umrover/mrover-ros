<template>
  <div>
    <div>
      <h3>{{ header }}</h3>
    </div>
    <table
      class="tableFormat"
      style="undefined;table-layout: fixed; width: auto"
    >
      <colgroup>
        <col
          v-for="i in moteusStateName.length + 1"
          :key="i"
          style="width: 30px"
        />
      </colgroup>
      <thead>
        <tr>
          <th class="tableElement tableHeader">Motor</th>
          <th
            v-for="(name, i) in moteusStateName"
            :key="i"
            class="tableElement"
          >
            {{ name }}
          </th>
        </tr>
        <tr>
          <th class="tableElement tableHeader">State</th>
          <td
            v-for="(state, i) in moteusStateState"
            :key="i"
            class="tableElement"
          >
            {{ state }}
          </td>
        </tr>
        <tr>
          <th class="tableElement tableHeader">Error</th>
          <td
            v-for="(error, i) in moteusStateError"
            :key="i"
            class="tableElement"
          >
            {{ error }}
          </td>
        </tr>
      </thead>
    </table>
  </div>
</template>

<script>
import ROSLIB from "roslib/src/RosLib";
import Vue from "vue";

export default {
  props: {
    header: {
      type: String,
      required: false,
      default: "Arm Moteus States"
    }
  },

  data() {
    return {
      moteusStateName: [],
      moteusStateState: [],
      moteusStateError: [],
      receivedBrushless: false,
      receivedBrushed: false
    };
  },

  created: function () {
    this.brushless_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/brushless_ra_data",
      messageType: "mrover/MotorsStatus"
    });
    
    this.brushed_sub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "/brushed_ra_data",
      messageType: "mrover/MotorsStatus"
    });
    
    this.brushless_sub.subscribe((msg) => {
      console.log("brushless:", msg);
      let moteus = msg.moteus_states;
      if (!this.receivedBrushless) {
        // if first time receiving brushless data, initialize the arrays
        this.receivedBrushless = true;
        let moteus = msg.moteus_states;
        for (let i = 0; i < moteus.name.length; ++i) {
          this.moteusStateName.push(moteus.name[i]);
          this.moteusStateState.push(moteus.state[i]);
          this.moteusStateError.push(moteus.error[i]);
        }
      } else {
        // else update existing keys
        this.updateMoteusObject(moteus);
      }
      this.sortMoteusStateDataByName();
    });

    this.brushed_sub.subscribe((msg) => {
      console.log("brushed:", msg);
      // if first time receiving brushed data, initialize the arrays
      let moteus = msg.moteus_states;
      if (!this.receivedBrushed) {
        this.receivedBrushed = true;
        for (let i = 0; i < moteus.name.length; ++i) {
          this.moteusStateName.push(moteus.name[i]);
          this.moteusStateState.push(moteus.state[i]);
          this.moteusStateError.push(moteus.error[i]);
        }
      } else {
        // else update existing keys
        this.updateMoteusObject(moteus);
      }
      this.sortMoteusStateDataByName();
    });
  },

  methods: {
    updateMoteusObject(msg) {
      // msg is a MoteusState object
      // Update only values with the names from the msg
      let name = [...this.moteusStateName];
      let state = [...this.moteusStateState];
      let error = [...this.moteusStateError];

      console.log("updateMoteusObjectNameBeforeLoop:", name);
      console.log("updateMoteusObjectStateBeforeLoop:", state);
      console.log("updateMoteusObjectErrorBeforeLoop:", error);

      for (let i = 0; i < msg.name.length; i++) {
        let index = name.findIndex((moteus) => moteus.name === msg.name[i]);
        console.log("UMOIndex:", index);
        if (index !== -1) {
          state[index] = msg.state[i];
          error[index] = msg.error[i];
        }
      }

      console.log("updateMoteusObjectNameAfterLoop:", name);
      console.log("updateMoteusObjectStateAfterLoop:", state);
      console.log("updateMoteusObjectErrorAfterLoop:", error);

      for (let i = 0; i < this.moteusStateName.length; ++i) {
        Vue.set(this.moteusStateName, i, name[i]);
        Vue.set(this.moteusStateState, i, state[i]);
        Vue.set(this.moteusStateError, i, error[i]);
      }
    },
    /*
    rostopic pub /brushless_ra_data mrover/MotorsStatus "name:
- ''
joint_states:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  name: ['']
  position: [0]
  velocity: [0]
  effort: [0]
moteus_states:
  name: ['joint_d','joint_e','joint_f']
  state: ['armed','armed','armed']
  error: ['none','none','none']" 
    */

    sortMoteusStateDataByName() {
      let name = this.moteusStateName;
      let state = this.moteusStateState;
      let error = this.moteusStateError;

      let sortedName = [...name].sort();
      let sortedState = [];
      let sortedError = [];

      for (let i = 0; i < sortedName.length; i++) {
        let index = name.indexOf(sortedName[i]);
        console.log("index:", index);
        sortedState.push(state[index]);
        sortedError.push(error[index]);
      }

      // console.log("sortedName:", sortedName);
      // console.log("sortedState:", sortedState);
      // console.log("sortedError:", sortedError);

      for (let i = 0; i < this.moteusStateName.length; ++i) {
        Vue.set(this.moteusStateName, i, sortedName[i]);
        Vue.set(this.moteusStateState, i, sortedState[i]);
        Vue.set(this.moteusStateError, i, sortedError[i]);
      }
    }
  }
};
</script>

<style scoped>
.wrap {
  display: inline-block;
  align-content: center;
}

.box {
  border-radius: 5px;
  padding: 10px;
  border: 1px solid black;
  text-align: right;
  vertical-align: top;
}

.tableFormat {
  border-collapse: collapse;
  border-spacing: 0;
}

.tableFormat td {
  border-color: black;
  border-style: solid;
  border-width: 1px;
  font-size: 13px;
  overflow: hidden;
  padding: 10px 5px;
  word-break: normal;
}

.tableFormat th {
  border-color: black;
  border-style: solid;
  border-width: 1px;
  font-size: 13px;
  font-weight: normal;
  overflow: hidden;
  padding: 10px 5px;
  word-break: normal;
}

.tableFormat .tableElement {
  border-color: inherit;
  text-align: center;
  vertical-align: top;
}

.tableElement.tableHeader {
  font-weight: bold;
}
</style>
