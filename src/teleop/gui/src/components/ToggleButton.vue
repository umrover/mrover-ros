<template>
  <label
    :for="id + '_button'"
    :class="{ active: currentState }"
    class="toggle__button"
  >
    <span v-if="currentState" class="toggle__label">{{ labelEnableText }}</span>
    <span v-if="!currentState" class="toggle__label">{{
      labelDisableText
    }}</span>

    <input :id="id + '_button'" v-model="checkedValue" type="checkbox" />
    <span class="toggle__switch"></span>
  </label>
</template>

<script>
export default {
  props: {
    labelEnableText: {
      type: String,
      required: true,
    },
    labelDisableText: {
      type: String,
      required: true,
    },
    id: {
      type: String,
      default: "primary",
    },
    currentState: {
      type: Boolean,
      required: true,
    },
  },
  computed: {
    checkedValue: {
      get() {
        return this.currentState;
      },
      set(newValue) {
        this.$emit("change", newValue);
      },
    },
  },
};
</script>

<style scoped>
.toggle__button {
  vertical-align: middle;
  user-select: none;
  cursor: pointer;
}

.toggle__button input[type="checkbox"] {
  opacity: 0;
  position: absolute;
  width: 1px;
  height: 1px;
}

.toggle__button .toggle__switch {
  display: inline-block;
  height: 12px;
  border-radius: 6px;
  width: 40px;
  background: #bfcbd9;
  box-shadow: inset 0 0 1px #bfcbd9;
  position: relative;
  margin-left: 10px;
  transition: all 0.25s;
}

.toggle__button .toggle__switch::after,
.toggle__button .toggle__switch::before {
  content: "";
  position: absolute;
  display: block;
  height: 18px;
  width: 18px;
  border-radius: 50%;
  left: 0;
  top: -3px;
  transform: translateX(0);
  transition: all 0.25s cubic-bezier(0.5, -0.6, 0.5, 1.6);
}

.toggle__button .toggle__switch::after {
  background: #4d4d4d;
  box-shadow: 0 0 1px #666;
}

.toggle__button .toggle__switch::before {
  background: #4d4d4d;
  box-shadow: 0 0 0 3px rgba(0, 0, 0, 0.1);
  opacity: 0;
}

.active .toggle__switch {
  background: #ffea9b;
  box-shadow: inset 0 0 1px #ffea9b;
}

.active .toggle__switch::after,
.active .toggle__switch::before {
  transform: translateX(40px - 18px);
}

.active .toggle__switch::after {
  left: 23px;
  background: #ffcb05;
  box-shadow: 0 0 1px #ffcb05;
}
</style>
