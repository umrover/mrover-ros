<template>
<div>
    <h3>Carousel Data</h3>
    <div class="box1">
        <Checkbox ref="open-loop" :name="'Open Loop'" @toggle="openLoop = !openLoop" />
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
                <button @pointerdown="velocity = -1*velocityScaleDecimal" @pointerup="velocity = 0" @mouseout="velocity = 0">Reverse</button>
                <button @pointerdown="velocity = velocityScaleDecimal" @pointerup="velocity = 0" @mouseout="velocity = 0">Forward</button>
                <input id="myRange" v-model="velocityScale" type="range" min="0" max="100">Velocity Scaling: {{velocityScale}}%</input>
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

    components: {
        Checkbox
    },
    data() {
        return {
            openLoop: false,
            velocity: 0,
            site: "A",
            velocityScale: 100,

            carousel_pub: null
        }
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

    created: function () {
        this.carousel_pub = new ROSLIB.Topic({
            ros: this.$ros,
            name: 'carousel_cmd',
            messageType: 'mrover/Carousel'
        });
        this.carousel_pub.publish(this.messageObject)
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
