<template>
    <div class="wrap box">
        <p>{{name}} ID: {{id}}</p>
        <p>Stream: {{stream}}</p>
        <label for="quality">Quality:</label>
        <select class="box" id="quality" v-model="selectedQuality" @change="changeQuality()">
          <option value="0">Low</option>
          <option value="1">Medium</option>
          <option value="2">High</option>
        </select>
    </div>
  </template>
  
  <script>
  
  export default {
    data() {
      return {
        selectedQuality: "1"
      }
    },

    props: {
        name: {
            type: String,
            required: true
        },
        id: {
            type: Number,
            required: true
        },
        stream: {
            type: Number,
            required: true
        }
    },


    methods: {
      changeQuality: function(){
        this.$emit('newQuality', {index: this.id, value: parseInt(this.selectedQuality)});
      }
    }

  }
  </script>
  
  <style scoped>

  .box {
    border-radius: 10px;
    border: 1px solid var(--shadow-color);
    box-shadow: 2px 2px 15px var(--shadow-color), -2px -2px 15px var(--shadow-color);
  }
  .wrap {
    margin: 10px;
    padding: 10px;
  }

  .wrap > * {
    margin: 5px 0 5px 0;
  }

  p {
    margin: 0px;
  }

  #quality {
    background-color: white;
  }

  </style>