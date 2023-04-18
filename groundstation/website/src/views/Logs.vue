<script lang="ts">
import { inject, defineComponent } from "vue";
import { socketProvider } from "@/plugins/socket";
import type { Socket } from "socket.io-client";
import { SERVER_URL } from "@/common/constants";
import type { Log, Mission } from "@/common/interfaces";

const MAX_LOG = 30;

export default defineComponent({
  name: "Logs",
  data() {
    return {
      socket: inject(socketProvider) as Socket,
      logs: [] as Log[],
      showFullLog: false,
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
    <div class = pageHeader>
      <h1 class="logs">Logs</h1>
      <p class="actions">
        <select v-model="mission_id">
          <option :value="0">Direct</option>
          <option
            v-for="mission in missions"
            :value="mission.id"
            :key="mission.id"
          >
            Mission #{{ mission.id }}
          </option>
        </select>
        <button @click="onPrev" :disabled="mission_id == 0">&larr;</button>
        <button @click="onNext">&rarr;</button>
        <button @click="showFullLog = !showFullLog">
          {{ showFullLog ? "▲" : "▼" }}
          {{ showFullLog ? "Afficher Moins" : "Afficher Plus" }}
        </button>
      </p>
    </div>
    <table>
      <tr>
        <th v-if = "logs.length !== 0">Temps</th>
        <th v-if = "logs.length !== 0">Robot</th>
        <th v-if = "logs.length !== 0">Évènement</th>
        <th v-if = "logs.length !== 0">Information</th>
      </tr>
      <tr v-for="log in logs" :key="log.id">
        <td>{{ new Date(log.time).toLocaleTimeString("it-IT") }}</td>
        <td>{{ log.robot }}</td>
        <td>{{ log.category }}</td>
        <td class="log-data">
          <div v-for="(line, index) in log.data.split('\n')" :key="index">
            <div v-if="line.length <= 50 || showFullLog">
              {{ line }}
            </div>
            <div v-else>{{ line.slice(0, 50) }}...</div>
          </div>
        </td>
      </tr>
    </table>
  </div>
</template>

<style scoped>
.log-data {
  white-space: pre;
}
.actions > button {
  margin-left: 10px;
}

.pageHeader{
  display: flex;
}

.logs{
  flex:1;
}

.actions{
  text-align: right;
  flex:1;
}

table {
  border-collapse: collapse;
}
td {
  border-top: 1px solid black;
  border-bottom: 1px solid black;
}
</style>
