<template>
    <div class="wrap">
      <div v-if="!disabled" class="wrap-button">
        <button :class="[color]" @click="toggleAndEmit()">
          <span class="white-text"
            >{{ name }}: {{ active ? "\u2611" : "\u2610" }}</span
          >
        </button>
      </div>
      <div v-else class="wrap-button button-disabled">
        <button :class="[color]">
          <span class="white-text"
            >{{ name }}: {{ active ? "\u2611" : "\u2610" }}</span
          >
        </button>
      </div>
    </div>
  </template>
  
  <script lang = "ts">
  export default {
    props: {
      name: {
        type: String,
        required: true,
      },
      disabled: {
        type: Boolean,
        default: false,
        required: false,
      },
    },
    data() {
      return {
        active: false,
      };
    },
  
    computed: {
      color: function () {
        return this.active ? "green" : "red";
      },
    },
  
    methods: {
      toggle: function () {
        this.active = !this.active;
      },
  
      toggleAndEmit: function () {
        this.toggle();
        this.$emit("toggle", this.active);
      },
    },
  };
  </script>
  
  <!-- <style scoped>
  .wrap {
    display: flex;
  }
  
  .wrap-button {
    display: flex;
    align-items: center;
    padding: 1px;
  }
  
  .wrap-button button{
    cursor: pointer;
  }
  
  .button-disabled {
    opacity: 0.5;
  }
  
  .green {
    background-color: green;
  }
  
  .green:hover{
    background-color: #00ff008e;
  }
  
  .red {
    background-color: red;
  }
  
  .red:hover{
    background-color: #ff000073;
  }
  
  .white-text {
    color: white;
  }
  </style> -->