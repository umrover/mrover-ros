<template>
  <div class="wrap">
    <div>
      <h3>Power Distribution Board</h3>
    </div>
    <div class="table">
      <table class="tableFormat" style="undefined;table-layout: fixed">
        <colgroup>
          <col style="width: 80px">
          <col style="width: 100px">
          <col style="width: 100px">
        </colgroup>
        <thead>
          <tr>
            <td class = "tableElement"></td>
            <td class = "tableElement">Temperature</td>
            <td class = "tableElement">Current</td>
          </tr>
        </thead>
        <tbody>
          <template v-for="(item, i) in pdb_data">
          <tr>
              <td class = "tableElement">{{voltage[i]}}</td>
              <td class = "tableElement" v-bind:style="{color:item.temp.color}">{{item.temp.val.toFixed(2)}}°C</td>
              <td class = "tableElement" v-bind:style="{color: item.current.color}">{{item.current.val.toFixed(2)}}A</td>
          </tr>
          </template>
        </tbody>
      </table>
    </div>
  </div>
  </template>

  <script>
  import ROSLIB from "roslib"
  const pdb_temp_limit = 100
  const pdb_current_limits = [//TBD
    2,
    8,
    8
  ]
  export default {
    data() {
      return {
        voltage: ["3.3V","5V","12V"],
        pdb_data: [
          {
            temp: {val: 0.0, color: "grey"},
            current: {val: 0.0, color: "grey"},
          },
          {
            temp: {val: 0.0, color: "grey"},
            current: {val: 0.0, color: "grey"},
          },
          {
            temp: {val: 0.0, color: "grey"},
            current: {val: 0.0, color: "grey"},
          }
        ],
      }
    },
    created: function() {
      this.pdb_data_sub= new ROSLIB.Topic({
        ros: this.$ros,
        name:'/diagnostic_data',
        messageType: 'mrover/Diagnostic'
      });
      this.pdb_data_sub.subscribe((msg)=> {
        for(let i=0;i<this.pdb_data.length;i++){
          this.updateVal((this.pdb_data[i]).temp, msg.temperatures[i], pdb_temp_limit)
          this.updateVal(this.pdb_data[i].current, msg.currents[i], pdb_current_limits[i])
        }
      })
    },
    methods: {
      updateVal: function(struct, val, threshold) {
        struct.val = val
        if (val > threshold || val < 0.0) {
          struct.color = "red"
        }
        else {
          struct.color = "black"
        }
      }
    }
  }
  </script>

  <style scoped>
    .tables {
      display: flex;
    }
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
      padding: 10px;
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
    .bold{
      font-weight: bold;
      border: 2px solid black;
    }
    .tableFormat .tableElement {
      border-color: inherit;
      text-align: center;
      vertical-align: top
    }
  </style>