<template>
    <div>
        <div class="wrapper">
            <div class="header">
                <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
                <h1>ROS Echo</h1>
                <div class="spacer"></div>
            </div>

            <div class="box pages">

                <label for="topic">Available Topics:</label>
                <ul id="topic">
                    <li v-for="topic in topics" :key="topic.name">
                    <input type="checkbox" :id="topic" :value="topic" @change="addType()" v-model="selectedTopics">
                    <label :for="topic">{{ topic.name }}</label>
                    </li>
                </ul>

                <table>
                    <tr>
                        <td class="box" v-for="c in cols">
                            {{ c.name }}
                            <button id="mute" class="box button" type="button" v-on:click="mute(c)">Mute</button>
                        </td>
                    </tr>
                    <tr>
                        <td class="box" v-for="c in cols">
                            <p id="feed" v-if="!c.muted" v-for="message in c.messages"> {{message}} </p>
                        </td>
                    </tr>
                </table>

            </div>
        </div>
    </div>
  </template>
  
  <script>
  
  import ROSLIB from "roslib"
  
  let interval;

  export default {
    name: 'ROSEcho',
    mounted() {
        this.populateTopics()
    },
    data() {
        return {
            topics: [],
            selectedTopics : [],
            feed: [],
            cols: []
        }
    },
    

    methods: {
        populateTopics : function(){
            var topicsClient = new ROSLIB.Service({
              ros : this.$ros,
              name : '/rosapi/topics',
              serviceType : 'rosapi/Topics'
            });

            var request = new ROSLIB.ServiceRequest();
            topicsClient.callService(request, (result) => {
                for(var i = 0; i < result.topics.length; i++){
                    this.topics.push(new ROSLIB.Topic({
                    ros: this.$ros,
                    name: result.topics[i],
                    messageType: result.types[i]
                    }));
                }
            });

        },

        addType : function(){
            //Add newly added topics to the column display and subscribe
            this.selectedTopics.forEach(prev => {
                const found = this.cols.find(newTopic => newTopic.name === prev.name);
                if(!found){
                    prev.subscribe((msg) => {
                        var topicCol = this.cols.find(c => c.name === prev.name);
                        if(topicCol) topicCol.messages.push(JSON.stringify(msg, null, '\t'));
                        if(topicCol.messages.length > 3) topicCol.messages.splice(0,1);
                    });
                    this.cols.push({name: prev.name, messages: [], muted: false});
                }
            });
            
            //Remove topics from column display if unchecked and unsubscribe
            this.cols.forEach(prev => {
                const found = this.selectedTopics.find(newTopic => newTopic.name === prev.name);
                if(!found){
                    var index = this.cols.findIndex(obj => obj === prev);
                    this.selectedTopics[index].unsubscribe();
                    this.cols.splice(index, 1);
                }
            });
        },

        mute: function(c) {
            c.muted = !c.muted;
            console.log(c.muted)
        }

    },
  
  
  beforeDestroy: function () {
  },
  
  created: function () {
    interval = window.setInterval(() => {
        var topicsClient = new ROSLIB.Service({
              ros : this.$ros,
              name : '/rosapi/topics',
              serviceType : 'rosapi/Topics'
            });

            var request = new ROSLIB.ServiceRequest();
            topicsClient.callService(request, (result) => {
                if(this.topics.length != result.topics.length){
                    this.topics.push(new ROSLIB.Topic({
                        ros: this.$ros,
                        name: result.topics[result.topics.length-1],
                        messageType: result.types[result.topics.length-1]
                    }));
                }
            });
    }, 1000);
  },
  
  }
  </script>
  
  <style scoped>

    ul {
      list-style-type: none;
      white-space: pre-wrap;
      list-style-position: outside;
    }

    .box {
        background-color: white;
        border-radius: 5px;
        padding: 10px;
        border-color: rgba(236, 236, 236, 0.966);
        box-shadow: 2px 2px 15px rgba(236, 236, 236, 0.966), -2px -2px 15px rgba(236,236,236,0.966);
    }

    .header {
        grid-area: header;
        display: flex;
        align-items: center;
        box-shadow: 0px 10px 8px -4px rgba(236, 236, 236, 0.966);
    }

    .header h1 {
        margin-left: 5px;
    }

    .pages {
        /* grid-area: pages;
        border: black solid 1px;
        border-radius: 5px;
        background-color: lightgray; */
        margin: 15px;
    }

    img {
        border: none;
        border-radius: 0px;
    }

    .wrapper {
        display: grid;
        grid-gap: 10px;
        grid-template-columns: 1fr;
        grid-template-rows: 60px 1fr;
        grid-template-areas: "header" "pages";
        font-family: "Arial";
        height: auto;
    }

    #mute {
        width: 80px;
        height: 40px;
        background-color: rgb(132, 169, 224);
        color: rgb(255, 255, 255);
        font-family: "Arial";
        font-size: medium;
        border-radius: 10px;
        border-color: transparent;
    }

    #mute:hover {
      background-color: rgb(116, 150, 201);
    }

    #mute:active {
      /* box-shadow: inset 0px 5px 5px rgb(142, 143, 145); */
      background-color: rgb(92, 124, 172);
    }

    table {
        width: 100%;
        table-layout: fixed;
    }

    td {
        vertical-align: top;
        text-align: left;
    }

    .button {
        float: right;
        text-align: center;
    }
    
    #feed {
        white-space: pre-wrap;
    }
    
  </style>