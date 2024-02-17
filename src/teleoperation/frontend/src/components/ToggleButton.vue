<template>
  <div class="form-check form-switch">
    <label :class="{ active: currentState }" class="form-check-label pointer" :for="id + '_button'">
      <span v-if="currentState">{{ labelEnableText }}</span>
      <span v-if="!currentState">{{ labelDisableText }}</span>
    </label>
    <input
      class="form-check-input pointer"
      type="checkbox"
      role="switch"
      v-model="checkedValue"
      :id="id + '_button'"
    />
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'

export default defineComponent({
  props: {
    labelEnableText: {
      type: String,
      required: true
    },
    labelDisableText: {
      type: String,
      required: true
    },
    id: {
      type: String,
      default: 'primary'
    },
    currentState: {
      type: Boolean,
      required: true
    }
  },
  emits: ['change'],
  computed: {
    checkedValue: {
      get() {
        return this.currentState
      },
      set(newValue: Boolean) {
        this.$emit('change', newValue)
      }
    }
  }
})
</script>
<style scoped>
.form-check-input:checked {
  background-color: var(--bs-primary);
  border-color: var(--bs-primary);
}

.pointer {
  cursor: pointer;
}
</style>
