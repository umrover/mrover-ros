<template>
  <div class="wrap">
      <div>
        <h3> End Effector UV </h3>
      </div>
      
    <div>
      <ToggleButton id="uv_end_effector" v-bind:currentState="endEffectorUVActive" labelEnableText="End Effector UV On" labelDisableText="End Effector UV Off" v-on:change="toggleEndEffectorUV()" />
    </div>
    <br>
    <div>
      <ToggleButton id="shutdown" v-bind:currentState="shutdownActive" labelEnableText="UV Auto shutoff On" labelDisableText="UV Auto shutoff Off" v-on:change="switchShutdown()"/>
    </div>
  </div>
</template>
    
<script>
  import ToggleButton from './ToggleButton.vue'
  import ROSLIB from 'roslib';

  export default {
    data () {
      return {
        endEffectorUVActive: false,
        shutdownActive: true,
        timeoutID: 0,
        scoopLimitActive: true,

        // Pubs and subs
        uvService: null
      }
    },

    components: {
      ToggleButton
    },

    watch: {
      endEffectorUVActive() {
        let request = new ROSLIB.ServiceRequest({
            enable: this.endEffectorUVActive
        });
        this.uvService.callService(request, (result) => {
            if (!result) {
                this.endEffectorUVActive = !this.endEffectorUVActive
                alert("Toggling End Effector UV failed.")
            }
        });
      }
    },

    methods: {
      switchShutdown: function() {
        this.shutdownActive = !this.shutdownActive
      },

      toggleEndEffectorUV: function () {
          this.endEffectorUVActive = !this.endEffectorUVActive
          if (this.endEffectorUVActive) {
            this.timeoutID = setTimeout(() => {
              if (this.endEffectorUVActive && this.shutdownActive) {
                this.endEffectorUVActive = false
              }
            }, 2 * 60000) // 2 minutes 
          }
          else {
            clearTimeout(this.timeoutID)
          }
      },

      toggleLimit: function () {
        this.scoopLimitActive = !this.scoopLimitActive
      }

    },

    created: function() {
      this.uvService = new ROSLIB.Service({
          ros: this.$ros,
          name: 'change_uv_led_end_effector_state',
          serviceType: 'mrover/ChangeDeviceState'
      });
    }
  }
</script>


<style scoped>
    .wrap {
        display: inline-block;
        align-content: center;
    }
</style>