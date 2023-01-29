<template>
<div class = "wrapper">
    <div class="box header">
        <img src="/static/mrover.png" alt="MRover" title="MRover" width="48" height="48" />
        <h1>ESW Debug</h1>
        <div class="spacer"></div>
        <div class="spacer"></div>
    </div>
    <DriveControls></DriveControls>
    <div class="box arm-controls light-bg">
      <ArmControls/>
    </div>
    <GimbalControls></GimbalControls>
    <div class="box drive-vel-data light-bg">
    <JointStateTable v-bind:jointStateData="jointState" v-bind:vertical="true"></JointStateTable>
    </div>
    <div class="box moteus light-bg">
    <MoteusStateTable v-bind:moteusStateData="moteusState"></MoteusStateTable>
    </div>
    <div class="box pdb light-bg">
    <PDBFuse></PDBFuse>
    </div>
    <div class="box cameras light-bg">
    <Cameras v-bind:primary="primary"></Cameras>
    <Checkbox ref="Primary" v-bind:name="'Primary Computer?'" v-on:toggle="primaryEnabled($event)"/> 
    </div>
</div>

</template>

<script>
import ROSLIB from "roslib"
import DriveControls from './DriveControls.vue';
import ArmControls from './ArmControls.vue';
import GimbalControls from './GimbalControls.vue';
import JointStateTable from './JointStateTable.vue';
import MoteusStateTable from './MoteusStateTable.vue';
import PDBFuse from './PDBFuse.vue';
import Cameras from './Cameras.vue';
import Checkbox from './Checkbox.vue'

export default {
    data() {
        return {
            jointState: {},
            moteusState: {},
            primary: false
        }
    },
    
    methods: {
        primaryEnabled: function (enabled){
            this.primary = enabled
        }
    },

    components: {
        DriveControls,
        ArmControls,
        GimbalControls,
        JointStateTable,
        MoteusStateTable,
        PDBFuse,
        Cameras,
        Checkbox
    },

    created: function () {
        this.brushless_motors = new ROSLIB.Topic({
            ros: this.$ros,
            name: 'drive_status',
            messageType: 'mrover/MotorsStatus'
        });

        this.brushless_motors.subscribe((msg) => {
            this.jointState = msg.joint_states
            this.moteusState = msg.moteus_states
        })
    }
}
</script>
<style scoped>
.wrapper{
    display: grid;
    grid-gap: 10px;
    grid-template-columns: auto auto;
    grid-template-rows: 60px 250px auto auto auto auto;
    grid-template-areas: "header header"
                         "arm-controls arm-controls"
                         "cameras drive-vel-data"
                         "moteus pdb";
                         
                         
    font-family: sans-serif;
    height: auto;
  }

  .header {
    grid-area: header;
    display: flex;
    align-items: center;
  }
  .cameras {
    grid-area: cameras;
  }

  .pdb {
    grid-area: pdb;
  }

  .drive-vel-data {
    grid-area: drive-vel-data;
  }

  .arm-controls{
    grid-area: arm-controls;
  }

  .moteus {
    grid-area: moteus;
  }

</style>