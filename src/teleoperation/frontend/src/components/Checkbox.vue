<template>
<<<<<<< HEAD
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
  
=======
  <div v-if="!disabled" class="wrap-button">
    <button :class="['btn', color]" @click="toggleAndEmit()">
      <span>{{ name }}: {{ active ? '\u2611' : '\u2610' }}</span>
    </button>
  </div>
  <div v-else class="wrap-button button-disabled">
    <button :class="['btn', color, 'disabled']">
      <span>{{ name }}: {{ active ? '\u2611' : '\u2610' }}</span>
    </button>
  </div>
</template>

>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
<script lang="ts">
import { defineComponent } from 'vue'

export default defineComponent({
<<<<<<< HEAD
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
        return this.active ? "btn-success" : "btn-danger";
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
});
</script>
  
<style scoped>
  .wrap-button {
    display: flex;
    align-items: center;
    justify-content: center;
    padding: 1px;
  }
  
  .button-disabled {
    opacity: 0.5;
  }
=======
  props: {
    name: {
      type: String,
      required: true
    },
    disabled: {
      type: Boolean,
      default: false,
      required: false
    }
  },
  data() {
    return {
      active: false
    }
  },

  computed: {
    color: function () {
      return this.active ? 'btn-success' : 'btn-danger'
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
})
</script>

<style scoped>
.wrap-button {
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 1px;
}

.button-disabled {
  opacity: 0.5;
}
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
</style>
