<template>
  <div>
    <div class="wrapper">
      <div class="header">
        <img
          src="/static/mrover.png"
          alt="MRover"
          title="MRover"
          width="48"
          height="48"
        />
        <h1>ROS Echo</h1>
        <div class="spacer"></div>
      </div>

      <div class="box pages">
        <label for="presets">Presets:</label>
        <select
          v-model="selectedPreset"
          class="box"
          required
          @change="changePreset()"
        >
          <option value="select" selected>Select a preset</option>
          <option v-for="option in Object.keys(presets)" :value="option">
            {{ option }}
          </option>
        </select>

        <label for="topic">Available Topics:</label>
        <ul id="topic">
          <li v-for="topic in topics" :key="topic.name">
            <input
              :id="topic"
              v-model="selectedTopics"
              type="checkbox"
              :value="topic"
            />
            <label :for="topic">{{ topic.name }}</label>
          </li>
        </ul>

        <table>
          <tr>
            <td v-for="c in cols" class="box">
              {{ c.name }}
              <button
                id="mute"
                class="box button"
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
import Vue from "vue";

export default {
  name: "ROSEcho",
  data() {
    return {
      presets: [],
      topics: [],
      selectedTopics: [],
      feed: [],
      cols: [],
      topicsService: null,
      serviceRequest: null,
      selectedPreset: "select",
    };
  },

  watch: {
    selectedTopics: function () {
      this.addType();
    },
  },
  mounted() {
    this.populateTopics();
  },

  created: function () {
    let presetList = new ROSLIB.Param({
      ros: this.$ros,
      name: "teleop/echo_presets",
    });

    presetList.get((values) => {
      Object.assign(this.presets, values);
    });
    this.presets["None"] = [];

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
        if (this.topics.length != temp.length) this.topics = temp;
      });
    },

    addType: function () {
      //method to handle checking/unchecking the topics
      //Add newly added topics to the column display and subscribe
      this.selectedTopics.forEach((prev) => {
        const found = this.cols.find((newTopic) => newTopic.name == prev.name);
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
      this.cols = this.cols.filter((prev) => {
        const found = this.selectedTopics.find(
          (newTopic) => newTopic.name === prev.name
        );
        if (!found) {
          var ind = this.topics.findIndex((obj) => obj.name === prev.name);
          this.topics[ind].unsubscribe();
          return false;
        }
        return true;
      });
    },

    mute: function (c) {
      c.muted = !c.muted;
    },

    changePreset: function () {
      //changes the selected topics to be the presets
      let presetTopics = this.presets[this.selectedPreset];
      for (var t in presetTopics) {
        var result = this.topics.find((obj) => obj.name == presetTopics[t]);
        Vue.set(this.selectedTopics, t, result);
      }
      //if "none" selected, then clear the selectedTopics array
      if (presetTopics.length == 0) {
        this.selectedTopics.splice(0, this.selectedTopics.length);
      }
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

.box {
  background-color: white;
  border-radius: 5px;
  padding: 10px;
  border-color: rgba(236, 236, 236, 0.966);
  box-shadow: 2px 2px 15px rgba(236, 236, 236, 0.966),
    -2px -2px 15px rgba(236, 236, 236, 0.966);
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
  color: rgb(255, 255, 255);
  font-family: "Arial";
  font-size: medium;
  border-radius: 10px;
  border-color: transparent;
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

.active {
  background-color: rgb(132, 169, 224);
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
