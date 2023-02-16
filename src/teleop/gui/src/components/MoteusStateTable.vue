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
          v-for="i in moteusStateData.name.length + 1"
          :key="i"
          style="width: 30px"
        />
      </colgroup>
      <thead>
        <tr>
          <th class="tableElement tableHeader">Motor</th>
          <th
            v-for="(name, i) in moteusStateData.name"
            :key="i"
            class="tableElement"
          >
            {{ name }}
          </th>
        </tr>
        <tr>
          <th class="tableElement tableHeader">State</th>
          <td
            v-for="(state, i) in moteusStateData.state"
            :key="i"
            class="tableElement"
          >
            {{ state }}
          </td>
        </tr>
        <tr>
          <th class="tableElement tableHeader">Error</th>
          <td
            v-for="(error, i) in moteusStateData.error"
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
import ROSLIB from "roslib";

export default {
  props: {
    moteusStateData: {
      type: Object,
      required: true
    },

    header: {
      type: String,
      required: false,
      default: "Motor Data"
    }
  },
  data() {
    return {
      motors: []
    };
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
