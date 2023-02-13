<template>
  <div>
    <h3>Carousel Data</h3>
    <div class="box1">
      <Checkbox
        ref="open-loop"
        :name="'Open Loop'"
        @toggle="openLoop = !openLoop"
      />
      <div class="controls">
        <div v-if="!openLoop">
          <label for="position">Rotate carousel to position: </label>
          <input v-model="site" type="radio" value="A" />A
          <input v-model="site" type="radio" value="B" />B
          <input v-model="site" type="radio" value="C" />C
        </div>
        <div v-else>
          <!-- Up and down arrows keys -->
          <p>Control with up and down arrow keys</p>
          <OpenLoopControl
            :forwards-key="38"
            :backwards-key="40"
            @velocity="velocity = $event"
          ></OpenLoopControl>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import ROSLIB from "roslib";
import Checkbox from "./Checkbox.vue";
import OpenLoopControl from "./OpenLoopControl.vue";

export default {
  components: {
    Checkbox,
    OpenLoopControl,
  },
  data() {
    return {
      openLoop: false,
      velocity: 0,
      site: "A",

      carousel_pub: null,
    };
  },

  computed: {
    messageObject: function () {
      const msg = {
        open_loop: this.openLoop,
        vel: this.velocity,
        site: this.site,
      };
      return new ROSLIB.Message(msg);
    },

    velocityScaleDecimal: function () {
      return this.velocityScale / 100;
    },
  },

  watch: {
    messageObject: function (msg) {
      this.carousel_pub.publish(msg);
    },
  },

  created: function () {
    this.carousel_pub = new ROSLIB.Topic({
      ros: this.$ros,
      name: "carousel_cmd",
      messageType: "mrover/Carousel",
    });
    this.carousel_pub.publish(this.messageObject);
  },
};
</script>

<style scoped>
.controls {
  display: inline-block;
  margin-top: 10px;
}

.box1 {
  text-align: left;
  vertical-align: top;
  display: inline-block;
}
</style>
