<template>
  <div class="wrap">
      <div>
        <h3> End Effector UV </h3>
      </div>
      
    <div>
      <ToggleButton id="uv_end_effector" v-bind:currentState="UV_endEffector" labelEnableText="End Effector UV On" labelDisableText="End Effector UV Off" v-on:change="toggleEndEffectorUV()" />
    </div>
    <br>
    <div>
      <ToggleButton id="shutdown" v-bind:currentState="shutdownActive" labelEnableText="UV Auto shutoff On" labelDisableText="UV Auto shutoff Off" v-on:change="switchShutdown()"/>
    </div>
  
    <!-- <div>
      <ToggleButton id="scoop_limit_switch" v-bind:currentState="scoopLimitActive" labelEnableText="Limit Switch On" labelDisableText="Limit Switch Off" v-on:change="toggleLimit()"/>
    </div> -->
  </div>
</template>
    
<script>
  import ToggleButton from './ToggleButton.vue'
  import ROSLIB from 'roslib';

  export default {
    data () {
      return {
        UV_endEffector: false,
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
      UV_endEffector() {
        let request = new ROSLIB.ServiceRequest({
            enable: this.UV_endEffector
        });
        this.uvService.callService(request, (result) => {
            if (!result) {
                this.UV_endEffector = !this.UV_endEffector
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
          this.UV_endEffector = !this.UV_endEffector
          if (this.UV_endEffector) {
            this.timeoutID = setTimeout(() => {
              if (this.UV_endEffector && this.shutdownActive) {
                this.UV_endEffector = false
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