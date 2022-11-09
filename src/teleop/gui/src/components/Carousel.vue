<template>
    <div>
        <h3>Carousel Data</h3>
        <div class="box1">
          <Checkbox ref="open-loop" v-bind:name="'Open Loop'" v-on:toggle="openLoop = !openLoop"/>
        <div class="controls">
          <div v-if="!openLoop">
            <label for="position">Rotate carousel to position: </label>
            <select v-model="site">
              <option>A</option>
              <option>B</option>
              <option>C</option>
            </select>
          </div>
          <div v-else>
            <!-- TODO: Add Event listeners for these buttons -->
            <button>Reverse</button>
            <button>Forward</button>
          </div>
        </div>
      </div>
    </div>
</template>

<script>
import ROSLIB from 'roslib'
import Checkbox from './Checkbox.vue'

let interval

export default {
    data() {
        return {
          openLoop: false,
          velocity: 0,
          site: "A",


          carousel_pub: null
        }
    },

    created: function (){
      this.carousel_pub = new ROSLIB.Topic({
      ros : this.$ros,
      name : 'carousel_cmd',
      messageType : 'mrover/Carousel'
    });
    this.carousel_pub.publish(this.messageObject)
    },

    computed:{
      messageObject: function(){
        const msg = {
          open_loop: this.openLoop,
          vel: this.velocity,
          site: this.site
        }
        return new ROSLIB.Message(msg)
      }
    },

    watch:{
      messageObject: function(msg){
        this.carousel_pub.publish(msg)
      }
    },

    components: {
      Checkbox
    }
}

</script>

<style scoped>


  .controls{
    display: inline-block;
    margin-top: 10px;
  }
  .box1 {
    text-align: left;
    vertical-align: top;
    display: inline-block;
  }
</style>
