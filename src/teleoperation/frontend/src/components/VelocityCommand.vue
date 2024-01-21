<template>
<<<<<<< HEAD
    <div>
      Velocity Command<br />
      Lin: {{ linear_x.toFixed(3) }} m/s<br />
      Ang: {{ angular_z.toFixed(3) }} rad/s
    </div>
  </template>
  
  <script lang="ts">
  
  export default {
    data() {
      return {
        linear_x: 0,
        angular_z: 0,
      };
    },
  
    created: function () {
=======
  <div>
    Velocity Command<br />
    Lin: {{ linear_x.toFixed(3) }} m/s<br />
    Ang: {{ angular_z.toFixed(3) }} rad/s
  </div>
</template>

<script lang="ts">
export default {
  data() {
    return {
      linear_x: 0,
      angular_z: 0
    }
  },

  created: function () {
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
    //   (this.cmd_vel_sub = new ROSLIB.Topic({
    //     ros: this.$ros,
    //     name: "cmd_vel",
    //     messageType: "geometry_msgs/Twist",
    //   })),
    //     this.cmd_vel_sub.subscribe((msg) => {
    //       // Only use linear x and angular z
    //       this.linear_x = msg.linear.x;
    //       this.angular_z = msg.angular.z;
    //     });
<<<<<<< HEAD
    },
  };
  </script>
  
  <style scoped></style>
=======
  }
}
</script>

<style scoped></style>
>>>>>>> 9720704e91b679513a528bf4bb0e7521bcdaec68
