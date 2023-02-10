<template>
    <div>
      <div class="wrap">
      <div class="page_header">
        <img src="/static/new_mrover.png" alt="MRover" title="MRover" width="185" height="53" />
        <h1>ROS Service</h1>
        <div class="spacer"></div>
      </div>
            
      <div class="pages box">
        <div class="requestCells">
            <label for="'service'">Service:</label>
            <select class="box" id="service" v-model="selectedService" @change="switchService()" required> 
            <option value="" selected>Select a service</option>
            <option v-for="option in service_options" v-bind:value="option">
            {{ option }}
            </option>
            </select>

          <textarea
            v-if="args != '' && args != '{}'"
            id="textarea"
            v-model="args"
            class="box"
          ></textarea>

          <p>{{ schema }}</p>
          <p v-if="error">JSON Syntax Error! Cannot send...</p>

          <button class="button" type="button" @click="sendArgs()">
            Send
          </button>
        </div>

        <p class="box responseCell">{{ response }}</p>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import '../assets/style.css';

const datatypes = [
  "bool",
  "int8",
  "uint8",
  "int16",
  "uint16",
  "int32",
  "uint32",
  "int64",
  "uint64",
  "float32",
  "float64",
  "string",
  "time",
  "duration",
];

export default {
  name: "ROSService",
  data() {
    return {
      service_options: [],
      selectedService: "",
      selectedType: "",
      args: "",
      schema: "",
      error: false,
      response: "",
    };
  },
  mounted() {
    this.populateServices();
  },

  methods: {
    populateServices: function () {
      //populates list of active services for the dropdown menu
      var topicsClient = new ROSLIB.Service({
        ros: this.$ros,
        name: "/rosapi/services",
        serviceType: "rosapi/Services",
      });

      var servicesSorted = [];
      var request = new ROSLIB.ServiceRequest();
      topicsClient.callService(request, (result) => {
        servicesSorted = result.services.sort();
        for (var r of servicesSorted) {
          this.service_options.push(r);
        }
      });
    },

    switchService: function () {
      //any time a service changes, it clears the display and displays the new schema (similar to ROSSend)
      this.args = "";
      this.schema = "";
      this.error = false;
      var serviceTypeClient = new ROSLIB.Service({
        ros: this.$ros,
        name: "/rosapi/service_type",
        serviceType: "ServiceType",
      });

      var request = new ROSLIB.ServiceRequest({
        service: this.selectedService,
      });
      serviceTypeClient.callService(request, (result) => {
        this.selectedType = result.type;
        getArgs(result.type);
      });

      const getArgs = (type) => {
        var serviceClient = new ROSLIB.Service({
          ros: this.$ros,
          name: "/rosapi/service_request_details",
          serviceType: "ServiceRequestDetails",
        });

        var request1 = new ROSLIB.ServiceRequest({ type: type });
        serviceClient.callService(request1, (result) => {
          const displayArgs = (arr) => {
            for (var i = 0; i < arr.fieldtypes.length; i++) {
              if (!datatypes.includes(arr.fieldtypes[i])) {
                this.args += '"' + arr.fieldnames[i] + '": ';
                if (arr.fieldarraylen[i] != -1) {
                  this.args += "[ ";
                }
                this.args += " {";
                ctr += 1;
                displayArgs(result.typedefs[ctr]);
                this.args += "\n}";
                if (arr.fieldarraylen[i] != -1) {
                  this.args += ", ]";
                }
              } else {
                this.args += '"' + arr.fieldnames[i] + '": ';
                if (arr.fieldarraylen[i] != -1) {
                  this.args += "[]";
                }
                if (i != arr.fieldnames.length - 1) {
                  this.args += ",\n";
                }

                this.schema +=
                  "\n" + arr.fieldtypes[i] + "\t" + arr.fieldnames[i];
              }
            }
          };
          var ctr = 0;
          displayArgs(result.typedefs[0]);
        });
      };
    },

    sendArgs: function () {
      //when button pressed, send service and show response
      this.error = false;
      if (this.args[0] != "{") this.args = "{" + this.args + "}";
      var parsed_args;
      try {
        if (this.args == "{}") parsed_args = "";
        else parsed_args = new ROSLIB.ServiceRequest(JSON.parse(this.args));
      } catch (e) {
        this.error = true;
      }

      if (!this.error) {
        var serviceClient = new ROSLIB.Service({
          ros: this.$ros,
          name: this.selectedService,
          serviceType: this.selectedType,
        });

        var request = new ROSLIB.ServiceRequest(parsed_args);
        serviceClient.callService(request, (result) => {
          this.response = result;
        });
      }
    },
  },
};
</script>

<style scoped>
p {
  width: 300px;
  white-space: pre-wrap;
}

.pages {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: repeat(2, 1fr);
  grid-template-rows: 1fr;
  grid-template-areas: "requestCell responseCell";
  margin: 10px;

}

#textarea {
  margin-top: 10px; 
  resize: none;
  height: auto;
  width: 75%;
  border-radius: 10px;
  font-family: "Arial";
  font-size: large;
}

#send {
    width: 100px;
    height: 50px;
    font-size: medium;
}

.responseCell {
  grid-area: responseCell;
  width: auto;
}

.responseCell {
  grid-area: responseCell;
  width: 100%;
}

.requestCells {
  grid-area: requestCell;
}
</style>
