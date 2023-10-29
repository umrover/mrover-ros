<template>
    <div v-if="!disabled" class="wrap-button">
      <button :class="['btn', color]" @click="toggleAndEmit()">
        <span
          >{{ name }}: {{ active ? "\u2611" : "\u2610" }}</span
        >
      </button>
    </div>
    <div v-else class="wrap-button button-disabled">
      <button :class="['btn', color, 'disabled' ]">
        <span
          >{{ name }}: {{ active ? "\u2611" : "\u2610" }}</span
        >
      </button>
    </div>
</template>
  
<script lang="ts">
import { defineComponent } from 'vue'

export default defineComponent({
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
<<<<<<< HEAD
        return this.active ? "green" : "red";
=======
        return this.active ? "btn-success" : "btn-danger";
>>>>>>> 0db5f0dc3bad5915fe968561b4d8e421d44dfe64
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
<<<<<<< HEAD
  };
  </script>
  
  <!-- <style scoped>
  .wrap {
    display: flex;
  }
  
=======
});
</script>
  
<style scoped>
>>>>>>> 0db5f0dc3bad5915fe968561b4d8e421d44dfe64
  .wrap-button {
    display: flex;
    align-items: center;
    padding: 1px;
  }
  
<<<<<<< HEAD
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
=======
  .button-disabled {
    opacity: 0.5;
  }
</style>
>>>>>>> 0db5f0dc3bad5915fe968561b4d8e421d44dfe64
