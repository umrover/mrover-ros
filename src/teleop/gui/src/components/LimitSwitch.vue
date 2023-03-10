<template>
    <div class="wrap">
      <ToggleButton
        :id="name"
        :current-state="limit_enabled"
        :label-enable-text= "name + ' On'"
        :label-disable-text= "name + ' Off'"
        @change="toggleLimitSwitch()"
      />
    </div>
</template>

<script>
    import ToggleButton from "./ToggleButton.vue";
    import ROSLIB from "roslib/src/RosLib";

    export default {
        props: {
            name: {
                type: String,
                required: true
            }
        },

        data() {
            return {
                limit_enabled: true
            }
        },

        components: {
            ToggleButton,
        },

        created: function () {
            this.limit_service = new ROSLIB.Service({
                ros: this.$ros,
                name: this.$props.state_name,
                serviceType: "mrover/ChangeDeviceState"
            });
        },

        methods: {
            toggleLimitSwitch: function () {
                this.limit_enabled = !this.limit_enabled;
                let request = new ROSLIB.ServiceRequest({
                    enable: this.limit_enabled
                });
                this.limit_service.callService(request, (result) => {
                    if (!result) {
                        this.limit_enabled = !this.limit_enabled;
                        alert("Toggling Limit Switch failed.");
                    }
                });
            }
        }

    }
</script>