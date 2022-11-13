<template>
    <div>
        <div class="wrapper">
            <div class="header">
            <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
            <h1>ROS Send</h1>
            <div class="spacer"></div>
            </div>
            
            <div class="pages">
                <label for="'topic'">Topic:</label>
                <select class="box" id="topic" v-model="selectedTopic" required> 
                <option value="" selected>Select a topic</option>
                <option v-for="option in topic_options" v-bind:value="option">
                {{ option }}
                </option>
                </select>

                <input v-if="selectedTopic == 'Custom topic'" v-model="customTopic" placeholder="Enter custom topic">

                <label for="'package'">Package:</label>
                <select class="box" id="package" v-model="selectedPackage" @change="switchPackage()" required> 
                <option value="" selected>Select a package</option>
                <option v-for="option in packages" v-bind:value="option">
                {{ option }}
                </option>
                </select>

                <label for="'type'">Type:</label>
                <select class="box" id="type" v-model="selectedType" @change="switchType()" required> 
                <option value="" selected>Select a message type</option>
                <option v-for="option in topic_types" v-bind:value="option">
                {{ option }}
                </option>
                </select>

                <textarea class="box" id="textarea" v-model="message"></textarea>

                <p> {{schema}} </p>
                <p v-if="error">JSON Syntax Error! Cannot send...</p>

                <button class="box" id="send" type="button" v-on:click="sendMessage()">Send</button>
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
    name: 'ROSSend',
    mounted() {
      this.populateTopics();
      this.populatePackages();
    },
    data() {
        return {
            topic_options: [],
            selectedTopic: '',
            topic_types: [],
            selectedType: '',
            packages: [],
            selectedPackage: '',
            message: '',
            customTopic: '',
            schema: '',
            error: false
        }
    },
    

    methods: {
        populateTopics: function() {  //populates all active topics for dropdown menu in addition to a Custom Topic option
          var topicsClient = new ROSLIB.Service({
              ros : this.$ros,
              name : '/rosapi/topics',
              serviceType : 'rosapi/Topics'
          });

          var topicsSorted = [];
          var request = new ROSLIB.ServiceRequest();
          topicsClient.callService(request, (result) => {
              topicsSorted = result.topics.sort();
              for(var r of topicsSorted){
                this.topic_options.push(r);
              }
          });

          this.topic_options.push('Custom topic');
        },

        populatePackages : function(){ //populates all active packages for the dropdown menu
            var packageClient = new ROSLIB.Service({
              ros : this.$ros,
              name : 'topic_services/fetch_packages',
              serviceType : 'FetchPackages'
            });

            var request = new ROSLIB.ServiceRequest();
            packageClient.callService(request, (result) => {
                for(var i = 0; i < result.packages.length; i++){
                    this.packages.push(result.packages[i]);
                }
            });

        },

        switchPackage : function(){
           //anytime a package is changed, it clears out the topics types 
           //available with that package and replaces it with the newly selected one
          this.topic_types = [];
          this.message = "";
          this.selectedType = '';
          this.schema = '';
            var topicTypeClient = new ROSLIB.Service({
              ros : this.$ros,
              name : 'topic_services/fetch_messages_from_package',
              serviceType : 'FetchMessageFromPackage'
            });

            var request1 = new ROSLIB.ServiceRequest({package: this.selectedPackage});
            topicTypeClient.callService(request1, (result) => {
              for(var i = 0; i < result.messages.length; i++){
                this.topic_types.push(result.messages[i]);
              }
            });
        },

        
        switchType: function(){
          //anytime a type changes, the display changes its schema to the new type. 
          //It's recursive so any non-primitive data type will be rooted down to its basic components. Also handles showing arrays
          this.message = "";
          this.schema = '';
          var messageClient = new ROSLIB.Service({
              ros : this.$ros,
              name : '/rosapi/message_details',
              serviceType : 'rosapi/MessageDetails'
          });

          var request2 = new ROSLIB.ServiceRequest({type: this.selectedType});
          messageClient.callService(request2, (result) => {

            const displayMessage = (arr) => {
              for(var i = 0; i < arr.fieldtypes.length; i++){
                if(!(datatypes.includes(arr.fieldtypes[i]))){

                    this.message += "\"" + arr.fieldnames[i] + "\": ";
                    if(arr.fieldarraylen[i] != -1){
                      this.message += "[ ";
                    }
                    this.message += " {";
                    ctr += 1; 
                    displayMessage(result.typedefs[ctr]);
                    this.message += "\n},\n";
                    if(arr.fieldarraylen[i] != -1){
                      this.message += ", ]";
                    }
                }
                else {
                  this.message += "\""+ arr.fieldnames[i] + "\": ";
                  if(arr.fieldarraylen[i] != -1){
                    this.message += "[]";
                  }
                  if(i != arr.fieldnames.length-1) {
                    this.message += ",\n";
                  }

                  this.schema += "\n" + arr.fieldtypes[i] + "\t" + arr.fieldnames[i];
                }

              }
            }

            var ctr = 0;
            displayMessage(result.typedefs[0]);
          });
        },

        sendMessage : function(){ //when the send button is pressed, it publishes that message
          this.error = false;
          var topic = this.selectedTopic;
          if(this.selectedTopic == 'Custom topic') topic = this.customTopic;
          if(this.message[0] != "{") this.message = "{" + this.message + "}";
          var publisher_msg;
          try {
            publisher_msg = new ROSLIB.Message(JSON.parse(this.message));
          } catch (e) {
              this.error = true;
          }
          if (!this.error){
            var publisher = new ROSLIB.Topic({
              ros: this.$ros,
              name: topic, 
              messageType: this.selectedType
            });
            publisher.publish(publisher_msg)        }
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

    .pages > * {
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
        height: 300px;
        width: 900px;
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

  </style>