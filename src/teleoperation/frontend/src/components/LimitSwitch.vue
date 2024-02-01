<template>
  <div class="wrap">
    <ToggleButton
      :id="name"
      :current-state="limit_enabled"
      :label-enable-text="name + ' On'"
      :label-disable-text="name + ' Off'"
      @change="toggleLimitSwitch()"
    />
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import ToggleButton from './ToggleButton.vue'
import { mapState, mapActions } from 'vuex'

export default defineComponent({
  components: {
    ToggleButton
  },
  props: {
    name: {
      type: String,
      required: true
    },
    switch_name: {
      type: String,
      required: true
    }
  },

  data() {
    return {
      limit_enabled: false
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == 'enable_limit_switch') {
        if (msg.result.length > 0) {
          this.limit_enabled = false
          for (var j = 0; j < msg.result.length; ++j) {
            alert('Toggling Limit Switch failed for' + msg.result[j])
          }
        }
      }
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
    toggleLimitSwitch: function () {
      this.limit_enabled = !this.limit_enabled
      this.sendMessage({
        type: 'enable_limit_switch',
        name: this.switch_name,
        enable: this.limit_enabled
      })
    }
  }
})
</script>
