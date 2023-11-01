<template>
    <div class="wrap">
      <span :class="['led', color]"></span>
      <div>
        <span class="name" v-if="show_name">{{ name }}</span>
      </div>
    </div>
  </template>
  
  <script>
  export default {
    props: {
      connected: {
        type: Boolean,
        required: true,
      },
      name: {
        type: String,
        required: true,
      },
      show_name: {
        type: Boolean,
        required: true,
      }
    },
    computed: {
      color: function () {
        return this.connected ? "green" : "red";
      },
    },
  };
  </script>
  
  <style scoped>
  .wrap {
    display: flex;
    align-items: center;
    height: 100%;
  }
  .led {
    width: 16px;
    height: 16px;
    border-radius: 8px;
    border: 1px solid;
  }
  .green {
    background-color: lightgreen;
  }
  .red {
    background-color: red;
  }
  .name {
    margin-left: 5px;
  }
  </style>