<template>
  <label
    :style="cssProps"
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
import "../assets/style.css";
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
    btnSize: {
      type: Number,
      default: 12
    }
  },
  emits: ["change"],
  computed: {
    checkedValue: {
      get() {
        return this.currentState;
      },
      set(newValue) {
        this.$emit("change", newValue);
      },
    },

    cssProps() {
      return {
        '--size': this.btnSize+"px"
      }
    }
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
  height: var(--size);
  border-radius: calc(var(--size)/2);
  width: calc(var(--size)*3.33);
  background: #bfcbd9;
  box-shadow: inset 0 0 1px #bfcbd9;
  position: relative;
  margin-left: 10px;
  vertical-align: center;
  transition: all 0.25s;
}
.toggle__button .toggle__switch::after,
.toggle__button .toggle__switch::before {
  content: "";
  position: absolute;
  display: block;
  height: calc(var(--size)*1.5);
  width: calc(var(--size)*1.5);
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
  background: var(--active-blue);
  box-shadow: inset 0 0 1px var(--active-blue);
}
.active .toggle__switch::after,
.active .toggle__switch::before {
  transform: translateX(calc(var(--size)*3.33) - calc(var(--size)*1.5));
}
.active .toggle__switch::after {
  left: calc(calc(var(--size)*3.33) - calc(var(--size)*1.5));
  background: var(--primary-blue);
  box-shadow: 0 0 1px var(--primary-blue);
}
</style>
