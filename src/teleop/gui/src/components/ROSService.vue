<template>
    <div>
        <div class="wrapper">
            <div class="header">
              <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
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

                <textarea v-if="args != '' && args != '{}'" class="box" id="textarea" v-model="args"></textarea>

                <p> {{schema}} </p>
                <p v-if="error">JSON Syntax Error! Cannot send...</p>

                <button class="box" id="send" type="button" v-on:click="sendArgs()">Send</button>

                </div>

                <p class="box responseCell">{{response}}</p>
            </div>
      </div>
    </div>
  </template>
  
  <script>
  
  import ROSLIB from "roslib"

  const datatypes = ['bool', 'int8', 'uint8', 'int16', 'uint16', 
                  'int32', 'uint32', 'int64', 'uint64', 'float32', 
                  'float64', 'string', 'time', 'duration']
  
  
  export default {
    name: 'ROSService',
    mounted() {
      this.populateServices();
    },
    data() {
        return {
            service_options: [],
            selectedService: '',
            selectedType: '',
            args: '',
            schema: '',
            error: false,
            response: ''
        }
    },
    

    methods: {
        populateServices: function() { //populates list of active services for the dropdown menu
          var topicsClient = new ROSLIB.Service({
              ros : this.$ros,
              name : '/rosapi/services',
              serviceType : 'rosapi/Services'
          });

          var servicesSorted = [];
          var request = new ROSLIB.ServiceRequest();
          topicsClient.callService(request, (result) => {
              servicesSorted = result.services.sort();
              for(var r of servicesSorted){
                this.service_options.push(r);
              }
          });

        },

        switchService : function(){ //any time a service changes, it clears the display and displays the new schema (similar to ROSSend)
            this.args = '';
            this.schema = '';
            this.error = false;
            var serviceTypeClient = new ROSLIB.Service({
              ros : this.$ros,
              name : '/rosapi/service_type',
              serviceType : 'ServiceType'
            });

            var request = new ROSLIB.ServiceRequest({service: this.selectedService});
            serviceTypeClient.callService(request, (result) => {
                this.selectedType = result.type;
                getArgs(result.type)
            });

            const getArgs = (type) => {
                var serviceClient = new ROSLIB.Service({
                ros : this.$ros,
                name : '/rosapi/service_request_details',
                serviceType : 'ServiceRequestDetails'
                });

                var request1 = new ROSLIB.ServiceRequest({type: type});
                serviceClient.callService(request1, (result) => {
                    const displayArgs = (arr) => {
                        for(var i = 0; i < arr.fieldtypes.length; i++){
                            if(!(datatypes.includes(arr.fieldtypes[i]))){

                                this.args += "\"" + arr.fieldnames[i] + "\": ";
                                if(arr.fieldarraylen[i] != -1){
                                this.args += "[ ";
                                }
                                this.args += " {";
                                ctr += 1; 
                                displayargs(result.typedefs[ctr]);
                                this.args += "\n}";
                                if(arr.fieldarraylen[i] != -1){
                                this.args += ", ]";
                                }
                            }
                            else {
                            this.args += "\""+ arr.fieldnames[i] + "\": ";
                            if(arr.fieldarraylen[i] != -1){
                                this.args += "[]";
                            }
                            if(i != arr.fieldnames.length-1) {
                                this.args += ",\n";
                            }

                            this.schema += "\n" + arr.fieldtypes[i] + "\t" + arr.fieldnames[i];
                            }

                        }
                    }
                    var ctr = 0;
                    displayArgs(result.typedefs[0]);
                });
            }

        },

        sendArgs : function(){ //when button pressed, send service and show response
          this.error = false;
          if(this.args[0] != "{") this.args = "{" + this.args + "}";
          var parsed_args;
          try {
            if(this.args == '{}') parsed_args = '';
            else parsed_args = new ROSLIB.ServiceRequest(JSON.parse(this.args));
          } catch (e){
              this.error = true;
          }
          
          if (!this.error){
            var serviceClient = new ROSLIB.Service({
                ros : this.$ros,
                name : this.selectedService,
                serviceType : this.selectedType
              });
  
              var request = new ROSLIB.ServiceRequest(parsed_args);
              serviceClient.callService(request, (result) => {
                  this.response = result;
              });
          }
        }
    },
  
  }
  </script>
  
  <style scoped>

    p {
      width: 300px;
        white-space: pre-wrap;
    }

    .box {
        background-color: white;
        border-radius: 5px;
        padding: 10px;
        border-color: rgba(236, 236, 236, 0.966);
        box-shadow: 2px 2px 15px rgba(236, 236, 236, 0.966), -2px -2px 15px rgba(236,236,236,0.966);
    }

    .header {
        display: flex;
        align-items: center;
        box-shadow: 0px 10px 8px -4px rgba(236, 236, 236, 0.966);
    }

    .header h1 {
        margin-left: 5px;
    }

    .pages {
        display: grid;
        grid-gap: 10px;
        grid-template-columns: repeat(2, 1fr);
        grid-template-rows: 1fr;
        grid-template-areas:
        "requestCell responseCell";
        margin: 10px;
    }

    img {
        border: none;
        border-radius: 0px;
    }

    .wrapper {
        font-family: "Arial";
        height: auto;
    }

    #textarea {
        display: flex;
        resize: none;
        height: auto;
        width: auto;
        border-radius: 10px;
        font-family: "Arial";
        font-size: large;
        
    }

    #send {
        width: 100px;
        height: 50px;
        background-color: rgb(132, 169, 224);
        color: rgb(255, 255, 255);
        font-family: "Arial";
        font-size: medium;
        border-radius: 10px;
        border-color: transparent;
    }

    #send:hover {
      background-color: rgb(116, 150, 201);
    }

    #send:active {
      background-color: rgb(92, 124, 172);
    }

    .responseCell {
      grid-area: responseCell;
      width: 100%;
    }

    .requestCells {
      grid-area: requestCell;
    }

  </style>