<script lang="ts">
import { inject, defineComponent } from "vue";
import { socketProvider } from "@/plugins/socket";
import type { Socket } from "socket.io-client";
import { SERVER_URL } from "@/common/constants";
import type { Mission } from "@/common/interfaces";

export default defineComponent({
  name: "Missions",
  data() {
    return {
      socket: inject(socketProvider) as Socket,
      missions: [] as Mission[],
    };
  },
  methods: {
    async loadMissions() {
      let res = await fetch(SERVER_URL + "/api/missions");
      let missions = (await res.json()) as Mission[];
      this.missions = missions;
    },
    missionDuration(mission: Mission): string {
      let millis =
        new Date(mission.end || Date.now()).getTime() -
        new Date(mission.start).getTime();
      let duration = Math.floor(millis / 1000);
      return `${Math.floor(duration / 60)}m${duration % 60}s`;
    },
    round(v: number) {
      return Math.floor(v * 10) / 10;
    },
    onMissionUpdate(message: string) {
      let mission = JSON.parse(message) as Mission;
      if (this.missions.find((e: Mission) => e.id == mission.id))
        this.missions.shift();
      this.missions.unshift(mission);
    },
  },
  mounted() {
    this.loadMissions();
    this.socket.on("mission_update", this.onMissionUpdate);
  },
  unmounted() {
    this.socket.off("mission_update", this.onMissionUpdate);
  },
});
</script>
<template>
  <div>
    <h1>Missions</h1>
    <table v-for="mission in missions" :key="mission.id">
      <tr>
        <th>Début</th>
        <td>
          {{ new Date(mission.start).toLocaleString("ca-FR") }}
        </td>
      </tr>
      <tr>
        <th>Durée</th>
        <td>
          {{ missionDuration(mission) }}
        </td>
      </tr>
      <tr>
        <th>Robots</th>
        <td>
          <input type="checkbox" :checked="mission.has_rover" disabled /> Rover
          <input type="checkbox" :checked="mission.has_drone" disabled /> Drone
        </td>
      </tr>
      <tr v-if="mission.has_rover">
        <th>Distance parcourue (rover)</th>
        <td>
          {{ round(mission.distance_rover) }}
        </td>
      </tr>
      <tr v-if="mission.has_drone">
        <th>Distance parcourue (drone)</th>
        <td>
          {{ round(mission.distance_drone) }}
        </td>
      </tr>
      <tr>
        <th>Simulation</th>
        <td>
          <input type="checkbox" :checked="mission.is_sim" disabled />
        </td>
      </tr>
    </table>
  </div>
</template>

<style scoped>
table {
  background-color: #d9c1b9;
  font-size: 20px;
  font-family: "Roboto", sans-serif;
  padding: 10px 20px;
  border: none;
  border-radius: 5px;
  box-shadow: 0px 0px 10px rgba(0, 0, 0, 0.2);
  width: 100%;
  margin-bottom: 15px;
}
table th {
  text-align: right;
  white-space: nowrap;
}
table td {
  white-space: nowrap;
  width: 100%;
  padding-left: 8px;
}
</style>
