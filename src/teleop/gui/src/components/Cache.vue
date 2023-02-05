<template>
<div>
    <h3>Cache Controls</h3>
    <button @pointerdown="velocity = -1*velocityScaleDecimal" @pointerup="velocity = 0" @mouseout="velocity = 0">Reverse</button>
    <button @pointerdown="velocity = velocityScaleDecimal" @pointerup="velocity = 0" @mouseout="velocity = 0">Forward</button>
    <input id="myRange" v-model="velocityScale" type="range" min="0" max="100">Velocity Scaling: {{velocityScale}}%</input>

    <!-- Cache is open loop for now 11/9/22 -->
    <!-- TODO: Add Indicator for cache being opened or closed once we have closed loop -->
</div>
</template>

<script>
import ROSLIB from 'roslib'

export default {
    data() {
        return {
            velocity: 0,
            velocityScale: 100,

            cache_pub: null
        }
    },

    computed: {
        messageObject: function () {
            const msg = {
                name: ["cache"],
                position: [],
                velocity: [this.velocity],
                effort: []
            }
            return new ROSLIB.Message(msg)
        },

        velocityScaleDecimal: function () {
            return this.velocityScale / 100
        }
    },

    watch: {
        messageObject: function (msg) {
            this.cache_pub.publish(msg)
        }
    },
    created: function () {
        this.cache_pub = new ROSLIB.Topic({
            ros: this.$ros,
            name: 'cache_cmd',
            messageType: 'sensor_msgs/JointState'
        });
        this.cache_pub.publish(this.messageObject)
    },

}
</script>

<style scoped>
#circle {
    height: 20px;
    width: 20px;
    border-radius: 50%;
    background-color: green;
    margin-right: 2%;
}

#box-1 {
    margin-top: 5%;
    display: flex;
    flex-direction: row;
}
</style>
