<script lang="ts">
import { inject, defineComponent } from "vue";
import { socketProvider } from "@/plugins/socket";
import type { Socket } from "socket.io-client";
import { SERVER_URL } from "@/common/constants";
import axios from "axios";
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
      let res = await axios.get(
        SERVER_URL +
          `/api/logs?mission=${this.mission_id}&start_id=${this.start_id}`
      );
      let logs = res.data as Log[];
      this.logs = logs;
    },
    async loadMissions() {
      let res = await axios.get(SERVER_URL + "/api/missions");
      let missions = res.data as Mission[];
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
      <div class="actions">
        <div class="dropdown">
          <select v-model="mission_id" class="mobile-select">
            <option :value="0">Direct</option>
            <option
              v-for="mission in missions"
              :value="mission.id"
              :key="mission.id"
            >
              Mission #{{ mission.id }}
            </option>
          </select>
        </div>
        <div class="button-group">
          <button @click="onPrev" :disabled="mission_id == 0">&larr;</button>
          <button @click="onNext">&rarr;</button>
          <button @click="showFullLog = !showFullLog">
            {{ showFullLog ? "▲" : "▼" }}
          </button>
        </div>
      </div>
    </div>
    <table>
      <colgroup>
        <col style="width: 20%" />
        <col style="width: 20%" />
        <col style="width: 30%" />
        <col style="width: 30%" />
      </colgroup>
      <tr>
        <th v-if = "logs.length !== 0">Temps</th>
        <th v-if = "logs.length !== 0">Robot</th>
        <th v-if = "logs.length !== 0">Évènement</th>
        <th v-if = "logs.length !== 0" class="info">Information</th>
      </tr>
      <tr v-for="log in logs" :key="log.id">
        <td>{{ new Date(log.time).toLocaleTimeString("it-IT") }}</td>
        <td>{{ log.robot }}</td>
        <td>{{ log.category }}</td>
        <td class="log-data">
          <div
            v-for="(line, index) in log.data.split('\n')"
            :key="index"
            :data-test="`log-data-line-${index}`"
          >
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

.mobile-select{
  position: relative;
  align-self:baseline;
  size: 100%;
  flex:1;
  font-family: "Kanit", sans-serif;
}

.dropdown {
  display: inline-block;
  vertical-align: middle;
  margin-right: 10px;
  position: relative;
}

.pageHeader{
  display: flex;
  justify-content: flex-end;
}

.pageHeader > * {
  height: 100%;
  line-height: 60px;
  align-items:end;
}

.logs{
  flex:1;
  margin-bottom: 10px;
}

.actions{
  display: flex;
  justify-content: flex-end;
  align-items: center;
}

table {
  border-collapse: collapse;
  width: 100%;
  table-layout: fixed;
}
td {
  border-top: 1px solid black;
  border-bottom: 1px solid black;
  font-size:14px;
  line-height: 1.5;
}

@media (max-width: 600px) {
  .mobile-select{
    width:100%;
    margin-bottom: 10px;
  }
  .button-group {
    display: flex; 
    justify-content: space-between;
    flex-direction: row;
    margin-bottom: 10px;
  }
  .actions{
    display: flex;
    flex-direction: column;
    text-align: center;
  }
  .pageHeader{
    display: flex;
    flex-direction: column;
    align-items:self-start;
  }  
}
</style>
