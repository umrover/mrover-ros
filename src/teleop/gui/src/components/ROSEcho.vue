<template>
    <div class="wrap">
      <div class="page_header">
          <img src="/static/new_mrover.png" alt="MRover" title="MRover" width="185" height="53" />
          <h1>ROS Echo</h1>
          <div class="spacer"></div>
      </div>

      <div class="box pages">
        <label for="topic">Available Topics:</label>
        <ul class="box" id="topic">
          <li v-for="topic in topics" :key="topic.name">
            <input
              :id="topic"
              v-model="selectedTopics"
              type="checkbox"
              :value="topic"
              @change="addType()"
            />
            <label :for="topic">{{ topic.name }}</label>
          </li>
        </ul>

        <div class="table_scroll">
        <table>
          <tr>
            <td v-for="c in cols" class="box">
              {{ c.name }}
              <button
                id="mute"
                class="button"
                type="button"
                :class="[c.muted ? 'inactive' : 'active']"
                @click="mute(c)"
              >
                Mute
              </button>
            </td>
          </tr>
          <tr>
            <td v-for="c in cols" class="box">
              <p v-for="message in c.messages" v-if="!c.muted" id="feed">
                {{ message }}
              </p>
            </td>
          </tr>
        </table>
        </div>
      </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import '../assets/style.css';

export default {
  name: "ROSEcho",
  data() {
    return {
      topics: [],
      selectedTopics: [],
      feed: [],
      cols: [],
      topicsService: null,
      serviceRequest: null,
    };
  },
  mounted() {
    this.populateTopics();
  },

  created: function () {
    this.serviceClient = new ROSLIB.Service({
      ros: this.$ros,
      name: "/rosapi/topics",
      serviceType: "rosapi/Topics",
    });

    this.serviceRequest = new ROSLIB.ServiceRequest();

    window.setInterval(() => {
      this.populateTopics();
    }, 1000);
  },

  methods: {
    populateTopics: function () {
      //populates active topics to the display
      this.serviceClient.callService(this.serviceRequest, (result) => {
        let temp = [];
        for (var i = 0; i < result.topics.length; i++) {
          temp.push(
            new ROSLIB.Topic({
              ros: this.$ros,
              name: result.topics[i],
              messageType: result.types[i],
            })
          );
        }
        this.topics = temp;
      });
    },

    addType: function () {
      //method to handle checking/unchecking the topics
      //Add newly added topics to the column display and subscribe
      this.selectedTopics.forEach((prev) => {
        const found = this.cols.find((newTopic) => newTopic.name === prev.name);
        if (!found) {
          prev.subscribe((msg) => {
            var topicCol = this.cols.find((c) => c.name === prev.name);
            if (topicCol)
              topicCol.messages.push(JSON.stringify(msg, null, "\t"));
            if (topicCol.messages.length > 3) topicCol.messages.splice(0, 1);
          });
          this.cols.push({ name: prev.name, messages: [], muted: false });
        }
      });

      //Remove topics from column display if unchecked and unsubscribe
      this.cols.forEach((prev) => {
        const found = this.selectedTopics.find(
          (newTopic) => newTopic.name === prev.name
        );
        if (!found) {
          var index = this.cols.findIndex((obj) => obj === prev);
          this.selectedTopics[index].unsubscribe();
          this.cols.splice(index, 1);
        }
      });
    },

    mute: function (c) {
      c.muted = !c.muted;
    },
  },
};
</script>

<style scoped>
ul {
  list-style-type: none;
  white-space: pre-wrap;
  list-style-position: outside;
}

.wrap {
  height: 100%;
}

.pages {
  margin: 15px;
  height: 90%;
}

.button {
  float: right;
}

#topic {
  height: 30%;
  overflow-y: scroll;
}

#feed {
  white-space: pre-wrap;
}

.active {
  background-color: var(--primary-blue);
}

table {
  width: 100%;
  table-layout: fixed;
}

td {
  vertical-align: top;
  text-align: left;
}

.table_scroll {
  height: 60%;
  overflow-y: scroll;
}

.inactive {
  background-color: rgb(129, 141, 158);
}

.active:hover {
  background-color: rgb(116, 150, 201);
}

.active:active {
  background-color: rgb(92, 124, 172);
}

.inactive:hover {
  background-color: rgb(120, 130, 145);
}

.inactive:active {
  background-color: rgb(99, 106, 116);
}
</style>
