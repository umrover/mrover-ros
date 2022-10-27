<template>
  <div class="wrap-button">
    <button v-bind:class="[this.color]" v-on:click='toggleAndEmit()'>
      <span class='white-text'>{{name}}: <br>{{active ? '\u2611' : '\u2610'}}</span>
    </button>
  </div>
</template>

<script>
export default {
  data () {
    return {
      active: false
    }
  },

  props: {
    name: {
      type: String,
      required: true
    },
    color: {
      type: String,
      required: true
    }
  },


  methods: {
    toggle: function () {
      this.active = !this.active
    },

    toggleAndEmit: function () {
      this.toggle()
      this.$emit('toggle', this.active)
    }
  }
}
</script>

<style scoped>
  .wrap-button {
    display: block;
    height: 100%;
    width: 100%;
    align-items: center;
    padding: 1px;
  }

  .green {
    background-color: green;
  }

  .red {
    background-color: red;
  }

  .yellow {
    background-color: #FFD700;
  }

  .white-text{
    font-size: large;
    color: white;
  }

</style>
