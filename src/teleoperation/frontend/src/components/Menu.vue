<template>
  <div class="wrap">
    <div class="pages">
      <fieldset class="row">
        <JointStateTable :jointStateData="{}" :vertical="true" />
        <legend>Tasks</legend>
        <MenuButton link="/EDMTask" name="EDM Mission" />
        <MenuButton link="/ESTask" name="ES Mission" />
        <MenuButton link="/ISHTask" name="ISH GUI" />
        <MenuButton link="/SATask" name="Sample Acquisition GUI" />
        <MenuButton link="/AutonTask" name="Autonomy Mission" />
        <MenuButton link="/Control" name="Temp Controls GUI" />
        <MenuButton link="/ROSDebug" name="ROS Debug Tools" />
      </fieldset>
    </div>
  </div>
</template>

<script lang="ts">
import { inject } from 'vue'
import JointStateTable from "./JointStateTable.vue"
export default {
  created() {
    const ws: any = inject<WebSocket>('webSocketService');

    ws.send('Hello, WebSocket!');
  },
  components: {
    JointStateTable
  }
};

</script>

<style scoped>
.wrap {
  display: inline-block;
  align-content: center;
}
</style>