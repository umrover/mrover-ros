<template>
  <div class="shadow my-1 p-3 rounded">
    
    <div class="row justify-content-md-left">
      <div class="form-group col-md-4">
        Stream:
        <input
          v-model="selectedStream"
          type="number"
          min="0"
          max="3"
          class="form-control"
          @change="swapStream"
        />
        <!-- <button class="box" @click="swapStream()">Change stream</button> -->
      </div>
      <div class="form-group col-md-4">
        <label for="quality">Quality:</label>
        <select
          v-model="selectedQuality"
          type="number"
          min="0"
          max="4"
          class="form-control"
          id="quality"
          @change="changeQuality()"
        >
          <option v-for="i in numQuality" :key="i">{{ i - 1 }}</option>
        </select>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { mapActions, mapState } from 'vuex'

export default {
  props: {
    name: {
      type: String,
      required: true
    },
    id: {
      type: Number,
      required: true
    },
    stream: {
      type: Number,
      required: true
    }
  },
  data() {
    return {
      selectedQuality: '2',
      selectedStream: this.stream,
      prevStream: this.stream,
      numQuality: 5
    }
  },

  watch: {
    message(msg) {
      if (msg.type == 'max_resolution') {
        this.numQuality = msg.res
      }
    },
    stream: function () {
      this.prevStream = this.stream
      this.selectedStream = this.stream
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    changeQuality: function () {
      this.$emit('newQuality', {
        index: this.id,
        value: parseInt(this.selectedQuality)
      })
    },

    swapStream() {
      this.$emit('swapStream', {
        prev: this.prevStream,
        newest: this.selectedStream
      })
      this.prevStream = this.selectedStream
    }
  }
}
</script>

<style scoped></style>
