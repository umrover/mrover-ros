<template>
<div>
    <h3>Carousel Data</h3>
    <div class="box1">
        <Checkbox ref="open-loop" v-bind:name="'Open Loop'" v-on:toggle="openLoop = !openLoop" />
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
                <button v-on:pointerdown="velocity = -1*velocityScaleDecimal" v-on:pointerup="velocity = 0" v-on:mouseout="velocity = 0">Reverse</button>
                <button v-on:pointerdown="velocity = velocityScaleDecimal" v-on:pointerup="velocity = 0" v-on:mouseout="velocity = 0">Forward</button>
                <input type="range" min="0" max="100" id="myRange" v-model="velocityScale">Velocity Scaling: {{velocityScale}}%</input>
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
            velocityScale: 100,

            carousel_pub: null
        }
    },

    created: function () {
        this.carousel_pub = new ROSLIB.Topic({
            ros: this.$ros,
            name: 'carousel_cmd',
            messageType: 'mrover/Carousel'
        });
        this.carousel_pub.publish(this.messageObject)
    },

    computed: {
        messageObject: function () {
            const msg = {
                open_loop: this.openLoop,
                vel: this.velocity,
                site: this.site
            }
            return new ROSLIB.Message(msg)
        },

        velocityScaleDecimal: function () {
            return this.velocityScale / 100
        }
    },

    watch: {
        messageObject: function (msg) {
            this.carousel_pub.publish(msg)
        }
    },

    components: {
        Checkbox
    }
}
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
