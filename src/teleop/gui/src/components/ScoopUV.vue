<template>
    <div class="wrap">
        <div>
          <h3> Scoop UV Bulb </h3>
        </div>
    
      <div>
        <ToggleButton id="scoop_button" v-bind:currentState="scoopUVActive" labelEnableText="Scoop UV On" labelDisableText="Scoop UV Off" v-on:change="toggleUVBulb()"/>
      </div>
        
      <div>
        <ToggleButton id="shutdown" v-bind:currentState="shutdownActive" labelEnableText="UV Auto shutoff On" labelDisableText="UV Auto shutoff Off" v-on:change="switchShutdown()"/>
      </div>
    
      <div>
        <ToggleButton id="scoop_limit_switch" v-bind:currentState="scoopLimitActive" labelEnableText="Limit Switch On" labelDisableText="Limit Switch Off" v-on:change="toggleLimit()"/>
      </div>
    </div>
    </template>
    
    <script>
    import ToggleButton from './ToggleButton.vue'
    export default {
      data () {
        return {
          scoopUVActive: false,
          shutdownActive: true,
          timeoutID: 0,
          scoopLimitActive: true
        }
      },
      components: {
        ToggleButton
      },
      methods: {
        switchShutdown: function() {
          this.shutdownActive = !this.shutdownActive
        },
        toggleUVBulb: function() {
          this.scoopUVActive = !this.scoopUVActive
          if (this.scoopUVActive) {
            this.timeoutID = setTimeout(() => {
              if (this.scoopUVActive && this.shutdownActive) {
                this.scoopUVActive = false
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
      }
    }
</script>


<style scoped>
    .wrap {
        display: inline-block;
        align-content: center;
    }
</style>