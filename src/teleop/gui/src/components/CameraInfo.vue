<template>
    <div class="wrap box">
        <p>{{name}} ID: {{id}}</p>
        Stream: <input class="box" type='Number' min="0" max="3" v-model ='selectedStream'>
        <button class="box" v-on:click="swapStream()">Change stream</button>
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
        selectedQuality: "0",
        selectedStream: this.stream,
        prevStream: this.stream
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
      },

      swapStream(){
        this.$emit('swapStream', {prev: this.prevStream, newest: this.selectedStream});
        this.prevStream = this.selectedStream;
      }
    },

    watch: {
      stream: function(){
        this.prevStream = this.stream;
        this.selectedStream = this.stream;
      }
    }

  }
  </script>
  
  <style scoped>

  .box {
    border-radius: 5px;
    padding: 10px;
    border: 1px solid black;
  }
  .wrap {
    margin: 10px;
    padding: 10px;
  }

  .wrap > * {
    margin: 5px 0 5px 0;
  }

  </style>