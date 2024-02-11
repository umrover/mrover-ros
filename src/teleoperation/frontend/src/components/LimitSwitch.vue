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
      if (msg.type == 'enable_device_srv') {
        if (!msg.result) {
          this.limit_enabled = false
          alert('Toggling Limit Switch failed.')
        }
      }
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
    toggleLimitSwitch: function () {
      this.limit_enabled = !this.limit_enabled
      this.sendMessage({
        type: 'enable_device_srv',
        name: this.switch_name,
        enable: this.limit_enabled
      })
    }
  }
})
</script>
