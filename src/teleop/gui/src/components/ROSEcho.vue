<template>
    <div>
        <div class="wrapper">
            <div class="box header">
                <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
                <h1>ROS Echo</h1>
                <div class="spacer"></div>
            </div>

            <div class="pages">
                <input v-model="customTopic" placeholder="Enter topic">
                <button type="button" v-on:click="addTopic()">Add</button>

                <ul id="topic">
                    <li v-for="topic in topics" :key="topic">
                    <input type="checkbox" :id="topic" :value="topic" @change="addType()" v-model="selectedTopics">
                    <label :for="topic">{{ topic }}</label>
                    </li>
                </ul>

                <ul id="feed">
                    <li v-for="(msg, i) in feed" :key="i">
                    {{i+1}}: {{ msg }}
                    </li>
                </ul>
            </div>
        </div>
    </div>
  </template>
  
  <script>
  
  import ROSLIB from "roslib"
  
  
  export default {
    name: 'ROSEcho',
    mounted() {
        this.populateTopics()
    },
    data() {
        return {
            topics: [],
            types: [],
            selectedTopics : [],
            selectedTypes: [],
            feed: [],
            customTopic: ''
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
                var topicsSorted = result.topics.sort();
                var typesSorted = result.types.sort();
                for(var i = 0; i < topicsSorted.length; i++){
                    this.topics.push(topicsSorted[i]);
                    this.types.push(typesSorted[i]);
                }
            });

        },

        addType : function(){

            var topicTypeClient = new ROSLIB.Service({
              ros : this.$ros,
              name : '/rosapi/topic_type',
              serviceType : 'rosapi/TopicType'
            });

            var request1 = new ROSLIB.ServiceRequest({topic: this.selectedTopics[this.selectedTopics.length-1]});
            topicTypeClient.callService(request1, (result) => {
                this.selectedTypes.push(result.type);
            });

            var sub = new ROSLIB.Topic({
            ros: this.$ros,
            name: this.selectedTopics[this.selectedTopics.length-1],
            messageType: this.selectedTypes[this.selectedTopics.length-1]
            });

            sub.subscribe((msg) => {
                if(this.feed.length > 3){
                    this.feed.splice(0, 1);
                }
                this.feed.push(sub.name + ":\n" + JSON.stringify(msg, null, '\t'));
            });

            console.log(this.selectedTopics)
            
        },

        addTopic : function(){
            var topicTypeClient = new ROSLIB.Service({
              ros : this.$ros,
              name : '/rosapi/topic_type',
              serviceType : 'rosapi/TopicType'
            });

            var type;
            var request1 = new ROSLIB.ServiceRequest({topic: this.customTopic});
            topicTypeClient.callService(request1, (result) => {
                type = result.type;
            });

            var sub = new ROSLIB.Topic({
            ros: this.$ros,
            name: this.customTopic,
            messageType: type
            });

            sub.subscribe((msg) => {
                if(this.feed.length > 8){
                    this.feed.splice(0, 1);
                }
                this.feed.push(sub.name + ":\n" + JSON.stringify(msg, null, '\t'));
            });

            this.topics.push(this.customTopic);
            this.selectedTopics.push(this.customTopic)
            this.types.push(type);
            this.selectedTypes.push(type);
        }
    },
  
  
  beforeDestroy: function () {
  },
  
  created: function () {
    
  },
  
  }
  </script>
  
  <style scoped>

    #feed {
        color: orange
    }

    ul {
      list-style-type: none;
      white-space: pre-wrap;
      list-style-position: outside;
    }

    .box {
        border-radius: 5px;
        padding: 10px;
        border: 1px solid black;
    }

    .header {
        grid-area: header;
        display: flex;
        align-items: center;
    }

    .header h1 {
        margin-left: 5px;
    }

    .pages {
        grid-area: pages;
        border: black solid 1px;
        border-radius: 5px;
        background-color: lightgray;
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
        font-family: sans-serif;
        height: auto;
    }
  </style>