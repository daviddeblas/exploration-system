<script lang="ts">
import { ref, inject, defineComponent } from "vue";
import { socketProvider } from "@/plugins/socket";
import type { Socket } from "socket.io-client";

interface Log {
  id: number;
  mission_id: number;
  time: string;
  robot: string;
  category: string;
  data: string;
}

export default defineComponent({
  name: "Home",
  data() {
    return {
      socket: inject(socketProvider) as Socket,
      logs: [] as Log[],
    };
  },
  methods: {
    onLogger(message: string) {
      this.logs.push(JSON.parse(message) as Log);
    },
  },
  mounted() {
    this.socket.on("logger", this.onLogger);
  },
  unmounted() {
    this.socket.off("logger", this.onLogger);
  },
});
</script>

<template>
  <div>
    <h1>Logs</h1>
    <table>
      <tr>
        <th>Robot</th>
        <th>Évènement</th>
        <th>Information</th>
      </tr>
      <tr v-for="log in logs" key="log.id">
        <td>{{ log.robot }}</td>
        <td>{{ log.category }}</td>
        <td class="log-data">{{ log.data }}</td>
      </tr>
    </table>
  </div>
</template>

<style>
.log-data {
  white-space: pre;
}
</style>
