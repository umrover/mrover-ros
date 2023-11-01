<template>
    <div class="wrap">
        <div v-if="vertical">
            <div>
                <h3>Motor Data</h3>
            </div>
            <table class="table" style="table-layout: fixed; width: 745px">
                <colgroup>
                    <col style="width: 85px" />
                    <col style="width: 60px" />
                    <col style="width: 75px" />
                    <col style="width: 85px" />
                    <col style="width: 60px" />
                    <col style="width: 75px" />
                    <col style="width: 85px" />
                    <col style="width: 60px" />
                    <col style="width: 75px" />
                    <col style="width: 85px" />
                </colgroup>

                <thead>
                    <tr class="table-primary">
                        <th>Motor</th>
                        <th>Positon (m)</th>
                        <th>Velocity (m/s)</th>
                        <th>Effort (Nm)</th>
                    </tr>
                </thead>

                <tbody>
                    <tr v-for="(joint, index) in jointStateData.name" :key="index">
                        <td>{{ joint }}</td>
                        <td>
                            {{ (jointStateData.position[index] * radius_m).toFixed(3) }}
                        </td>
                        <td>
                            {{ (jointStateData.velocity[index] * radius_m).toFixed(3) }}
                        </td>
                        <td>
                            {{ jointStateData.effort[index].toFixed(3) }}
                        </td>
                    </tr>
                </tbody>
            </table>
        </div>

        <div v-else>
            <div>
                <h3>Motor Data</h3>
            </div>
            <table class="table" style="table-layout: fixed; width: 745px">
                <colgroup>
                    <col style="width: 85px" />
                    <col style="width: 60px" />
                    <col style="width: 75px" />
                    <col style="width: 85px" />
                    <col style="width: 60px" />
                    <col style="width: 75px" />
                    <col style="width: 85px" />
                    <col style="width: 60px" />
                    <col style="width: 75px" />
                    <col style="width: 85px" />
                </colgroup>

                <thead>
                    <tr>
                        <th>Motor</th>
                        <th v-for="name in jointStateData.name" class="tableElement">
                            {{ name.toFixed(3) }}
                        </th>
                    </tr>
                    <tr>
                        <th>Position (m)</th>
                        <td v-for="position in jointStateData.position" class="tableElement">
                            {{ (position * radius_m).toFixed(3) }}
                        </td>
                    </tr>
                    <tr>
                        <th>Velocity (m/s)</th>
                        <td v-for="velocity in jointStateData.velocity" class="tableElement">
                            {{ (velocity * radius_m).toFixed(3) }}
                        </td>
                    </tr>
                    <tr>
                        <th>Effort (Nm)</th>
                        <td v-for="effort in jointStateData.effort" class="tableElement">
                            {{ effort.toFixed(3) }}
                        </td>
                    </tr>
                </thead>
            </table>
        </div>
    </div>
</template>
  
<script lang="ts">
import { inject } from 'vue';


export default {
    props: {
        // Table will only render headers if these values are not passed w/ v-bind
        jointStateData: {
            type: Object,
            required: true,
        },

        vertical: {
            type: Boolean,
            required: true,
        },
    },

    data() {
        return {
            websocket: inject("webSocketService") as WebSocket,
            motors: [],
            radius_m: 0
        };
    },

    created: function () {
        this.websocket.onmessage = (msg) => {
            msg = JSON.parse(msg.data)
            if (msg.type == "joint_state") {
                this.name = msg.name;
                this.position = msg.position;
                this.velocity = msg.velocity;
                this.effore = msg.effort;
            }
        }
    }
};
</script>
  
<style scoped>
.wrap {
    display: inline-block;
    align-content: center;
}
</style>