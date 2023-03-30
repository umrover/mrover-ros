<template>
  <div class="wrap">
    <div>
      <div>
        <h3>Drive Moteus States</h3>
      </div>
      <table
        class="tableFormat"
        style="undefined;table-layout: fixed; width: 400px"
      >
        <colgroup>
          <col style="width: 20px" />
          <col style="width: 30px" />
          <col style="width: 30px" />
        </colgroup>
        <thead>
          <div>
            <DriveMoteusStuff
              class = "tableElement tableHeader"
              :header="'Motor'"
              :leftmotor="moteusStateData.name[indices.FrontLeft]"
              :rightmotor="moteusStateData.name[indices.FrontRight]"
            >
            </DriveMoteusStuff>
          </div>
          <div>
            <DriveMoteusStuff
              :header="'State'"
              :leftmotor="moteusStateData.state[indices.FrontLeft]"
              :rightmotor="moteusStateData.state[indices.FrontRight]"
            >
            </DriveMoteusStuff>
          </div>
          <div>
            <DriveMoteusStuff
              :header="'Error'"
              :leftmotor="moteusStateData.error[indices.FrontLeft]"
              :rightmotor="moteusStateData.error[indices.FrontRight]"
            >
            </DriveMoteusStuff>
          </div>
          <div>
            <DriveMoteusStuff
              :header="'Motor'"
              :leftmotor="moteusStateData.name[indices.MiddleLeft]"
              :rightmotor="moteusStateData.name[indices.MiddleRight]"
            >
            </DriveMoteusStuff>
          </div>
          <div>
            <DriveMoteusStuff
              :header="'State'"
              :leftmotor="moteusStateData.state[indices.MiddleLeft]"
              :rightmotor="moteusStateData.state[indices.MiddleRight]"
            >
            </DriveMoteusStuff>
          </div>
          <div>
            <DriveMoteusStuff
              :header="'Error'"
              :leftmotor="moteusStateData.error[indices.BackLeft]"
              :rightmotor="moteusStateData.error[indices.BackRight]"
            >
            </DriveMoteusStuff>
          </div>
          <div class="tableElement tableHeader">
            <DriveMoteusStuff
              :header="'Motor'"
              :leftmotor="moteusStateData.name[indices.BackLeft]"
              :rightmotor="moteusStateData.name[indices.BackRight]"
            >
            </DriveMoteusStuff>
            <div>
              <DriveMoteusStuff
                :header="'State'"
                :leftmotor="moteusStateData.state[indices.BackLeft]"
                :rightmotor="moteusStateData.state[indices.BackRight]"
              >
              </DriveMoteusStuff>
            </div>
            <div>
              <DriveMoteusStuff
                :header="'Error'"
                :leftmotor="moteusStateData.error[indices.BackLeft]"
                :rightmotor="moteusStateData.error[indices.BackRight]"
              >
              </DriveMoteusStuff>
            </div>
          </div>
        </thead>
      </table>
    </div>
  </div>
</template>

<script>
import DriveMoteusStuff from "./DriveMoteusRow.vue";
import ROSLIB from "roslib";

const state = {
  FrontLeft: 0,
  FrontRight: 1,
  MiddleLeft: 2,
  MiddleRight: 3,
  BackLeft: 4,
  BackRight: 5
};

export default {
  components: {
    DriveMoteusStuff
  },

  props: {
    moteusStateData: {
      type: Object,
      required: true
    }
  },
  data() {
    return {
      indices: state
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
