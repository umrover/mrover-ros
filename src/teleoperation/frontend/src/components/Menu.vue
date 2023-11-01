<template>
  <div class="wrap">
    <div class="pages">
      <fieldset class="row">
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
import MenuButton from './MenuButton.vue'
export default {
  created() {
    const ws: any = inject<WebSocket>('webSocketService');

    ws.send('Hello, WebSocket!');
  },
  components: {
    MenuButton
  }
};

</script>
<style scoped>
.wrap {
  display: inline-block;
  align-content: center;
}
</style>