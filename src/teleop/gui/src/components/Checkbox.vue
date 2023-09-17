<template>
  <div class="wrap-button">
    <button v-bind:class="[color]" v-on:click='toggleAndEmit()'>
      <span class='white-text'>{{name}}: {{active ? '\u2611' : '\u2610'}}</span>
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
    }
  },

  computed: {
    color: function () {
      return this.active ? 'green' : 'red'
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
    display: flex;
    align-items: center;
    padding: 1px;
  }

  .green {
    background-color: green;
  }

  .red {
    background-color: red;
  }

  .white-text{
      color: white;
  }

</style>
