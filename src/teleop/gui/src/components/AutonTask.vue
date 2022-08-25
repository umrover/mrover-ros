<template>
<div>
    <p>Auton</p>
    <button v-on:click="ros_msg()">Send Ros Message</button>
</div>
</template>

<script>
import ROSLIB from "roslib"

export default {
  data() {
    return {
      count: 0
    }
  },

  created: function () {
  },

  methods: {
    ros_msg(){
        console.log("Message Being Sent")
        var cmdVel = new ROSLIB.Topic({
            ros : this.$ros,
            name : '/cmd_vel',
            messageType : 'geometry_msgs/Twist'
        })

        var twist = new ROSLIB.Message({
            linear : {
                x : 0.5,
                y : 0.0,
                z : 0.0
            },
            angular : {
                x : 0.0,
                y : 0.0,
                z : 0.5
            }
        })

        cmdVel.publish(twist)
    }
  }
}
</script>

