<script lang="ts">
import { ref, inject, defineComponent } from "vue";
import { socketProvider } from "@/plugins/socket";
import type { Socket } from "socket.io-client";
import { SERVER_URL } from "@/common/constants";

interface Log {
  id: number;
  mission_id: number;
  time: string;
  robot: string;
  category: string;
  data: string;
}

interface Mission {
  id: number;
  start: string;
}

const MAX_LOG = 30;

export default defineComponent({
  name: "Home",
  data() {
    return {
      socket: inject(socketProvider) as Socket,
      logs: [] as Log[],
      missions: [] as Mission[],
      start_id: 0,
      mission_id: 0,
    };
  },
  methods: {
    onLogger(message: string) {
      if (this.mission_id != 0 || this.start_id != 0) return;
      let log = JSON.parse(message) as Log;
      if (this.logs.find((e: Log) => e.id == log.id)) return;
      this.logs.unshift(log);
      if (this.logs.length > MAX_LOG) {
        this.logs.pop();
      }
    },
    async onNext() {
      if (this.mission_id == 0 && this.logs.length > 0) {
        this.mission_id = this.logs[0].mission_id;
      }
      if (this.logs.length < MAX_LOG) return;
      let before = this.start_id;
      this.start_id = this.logs[this.logs.length - 1].id - 1;
      await this.loadLogs();
      if (this.logs.length == 0) {
        this.start_id = before;
        await this.loadLogs();
      }
    },
    async onPrev() {
      if (this.logs.length == 0) return;
      let before = this.start_id;
      this.start_id = this.logs[0].id + MAX_LOG;
      await this.loadLogs();
      if (this.logs.length == 0) {
        this.start_id = before;
        await this.loadLogs();
      }
    },
    async loadLogs() {
      let res = await fetch(
        SERVER_URL +
          `/api/logs?mission=${this.mission_id}&start_id=${this.start_id}`
      );
      let logs = (await res.json()) as Log[];
      this.logs = logs;
    },
    async loadMissions() {
      let res = await fetch(SERVER_URL + "/api/missions");
      let missions = (await res.json()) as Mission[];
      this.missions = missions;
    },
  },
  watch: {
    mission_id(mission: number) {
      if (mission == 0) this.start_id = 0;
      this.loadLogs();
    },
  },
  mounted() {
    this.socket.on("logger", this.onLogger);
    this.loadMissions();
    this.loadLogs();
  },
  unmounted() {
    this.socket.off("logger", this.onLogger);
  },
});
</script>

<template>
  <div>
    <h1>Logs</h1>
    <p class="actions">
      <select v-model="mission_id">
        <option :value="0">Direct</option>
        <option v-for="mission in missions" :value="mission.id">
          {{ mission.id }}
        </option>
      </select>
      <button @click="onPrev" :disabled="mission_id == 0">&larr;</button>
      <button @click="onNext">&rarr;</button>
    </p>
    <table>
      <tr>
        <th>Temps</th>
        <th>Robot</th>
        <th>Évènement</th>
        <th>Information</th>
      </tr>
      <tr v-for="log in logs" :key="log.id">
        <td>{{ new Date(log.time).toLocaleTimeString("it-IT") }}</td>
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
.actions > button {
  margin-left: 10px;
  width: 50px;
}
</style>
