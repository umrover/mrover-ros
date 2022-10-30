<template>
    <div class="wrap">
        <h3>Cameras</h3>
        <div class="input">
            Camera name: <input type='message' v-model ='cameraName'>
            Camera number: <input type='Number' min="0" max="9" v-model ='cameraIdx'>
            <button v-on:click="addCameraName()">Change name</button>
        </div>
        <div class="cameraselection">
            <CameraSelection class="cameraspace1" v-bind:camsEnabled="camsEnabled" v-bind:names="names" v-bind:numCams="numCams" v-on:cam_index="setCamIndex($event)"/>
        </div>
    </div>
</template>
  
<script>
    import CameraSelection from './CameraSelection.vue'
    import Checkbox from './Checkbox.vue'

    let interval;

    export default {
        data() {
            return {
                camsEnabled: [
                    false,
                    false,
                    false,
                    false,
                    false,
                    false,
                    false,
                    false,
                    false,
                ],
                names: [
                    "Camera 0",
                    "Camera 1",
                    "Camera 2",
                    "Camera 3",
                    "Camera 4",
                    "Camera 5",
                    "Camera 6",
                    "Camera 7",
                    "Camera 8"
                ],
                
                cameraIdx: 1,
                cameraName: ""
            }
        },

        beforeDestroy: function () {
            window.clearInterval(interval);
        },

        props: {
            numCams: {
                type: Number,
                required: true
            },
            mission: {
                type: String,
                required: true
            },
            channel: {
                type: String,
                required: true
            }
        },
        methods: {
            setCamIndex: function (index) {
                this.camsEnabled.splice(index, 1, !this.camsEnabled[index])
                this.sendCameras();
            },
            sendCameras: function() {
                this.$parent.publish('/cameras_mission', {
                    'type': 'Mission',
                    'name': this.mission
                })
                let ports = []
                for (let i = 0; i < this.camsEnabled.length && ports.length < this.numCams; i++) {
                    if (this.camsEnabled[i]) {
                    ports.push(i)
                    }
                }
                // The condition (i < 4) must match the length of port in GUICameras.
                for (let i = ports.length; i < 4; i++) {
                    ports.push(-1)
                }
                this.$parent.publish(this.channel, {
                    'type': 'GUICameras',
                    'port': ports,
                })
            },
            addCameraName: function() {
                this.names.splice(this.cameraIdx, 1, this.cameraName)
            }
        },
        components: {
            CameraSelection,
            Checkbox
        }
    }
    </script>

    <style>
    .wrap {
        display: grid;
        grid-gap: 10px;
        grid-template-columns: 1fr;
        grid-template-rows: 60px;
        grid-template-areas: "header";
        font-family: sans-serif;
        height: 100%;
    }

    .cameraselection {
        display: grid;
        grid-template-columns: 1fr 1fr;
        grid-template-areas: "cameraspace1"
    }

    .cam_buttons {
        height:20px;
        width:100px;
    }

    .box {
        border-radius: 5px;
        padding: 10px;
        border: 1px solid black;
    }

    img {
        border: none;
        border-radius: 0px;
    }

    .header {
        grid-area: header;
        display: flex;
        align-items: center;
    }

    .header h1 {
        margin-left: 5px;
    }

    .spacer {
        flex-grow: 0.8;
    }

    .comms {
        display: flex;
        flex-direction: column;
        align-items: flex-start;
    }

    .comms * {
        margin-top: 2px;
        margin-bottom: 2px;
        display: flex;
    }
    
    ul#vitals li {
        display: inline;
        padding: 0px 10px 0px 0px;
    }
</style>