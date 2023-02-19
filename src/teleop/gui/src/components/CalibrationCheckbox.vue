<template>
    <div class="wrap">
        <Checkbox
        :name="name">
        </Checkbox>
    </div>
</template>

<script>
import Checkbox from "./Checkbox.vue";

export default {
    props: {
        name: {
            type: String,
            required: true
        }
    },

    data() {
        return {
            enabled: true
        }
    },
    
    components: {
        Checkbox
    },
    
    created: function () {
        this.limit_service = new ROSLIB.Service({
            ros: this.$ros,
            name: "",
            serviceType: "mrover/CalibrateMotors"
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

<style>
    .wrap {
        padding: 2%;
    }

</style>