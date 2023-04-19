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
      sortBy: "start",
      sortDir: 1,
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
      let index = this.missions.findIndex((e: Mission) => e.id == mission.id);
      if (index >= 0) {
        this.missions[index] = mission;
      } else {
        this.missions.unshift(mission);
      }
    },
  },
  mounted() {
    this.loadMissions();
    this.socket.on("mission_update", this.onMissionUpdate);
  },
  unmounted() {
    this.socket.off("mission_update", this.onMissionUpdate);
  },
  computed: {
    sortedMissions() {
      if (this.sortBy == "duration") {
        return this.missions.sort((a, b) => {
          let aDuration =
            new Date(a.end || Date.now()).getTime() -
            new Date(a.start).getTime();
          let bDuration =
            new Date(b.end || Date.now()).getTime() -
            new Date(b.start).getTime();
          if (aDuration > bDuration) return -1 * this.sortDir;
          if (bDuration > aDuration) return 1 * this.sortDir;
          return 0;
        });
      } else if (this.sortBy == "start") {
        return this.missions.sort((a, b) => {
          let aTime = new Date(a.start).getTime();
          let bTime = new Date(b.start).getTime();
          if (aTime > bTime) return -1 * this.sortDir;
          if (bTime > aTime) return 1 * this.sortDir;
          return 0;
        });
      }
      return this.missions.sort((a, b) => {
        let aa = a as any[string];
        let bb = b as any[string];
        if (aa[this.sortBy] > bb[this.sortBy]) return -1 * this.sortDir;
        if (bb[this.sortBy] > aa[this.sortBy]) return 1 * this.sortDir;
        return 0;
      });
    },
  },
});
</script>
<template>
  <div>
    <h1>Missions</h1>
    <p class="actions">
      <select v-model="sortBy">
        <option value="start">Début</option>
        <option value="duration">Durée</option>
        <option value="distance_rover">Distance parcourue (rover)</option>
        <option value="distance_drone">Distance parcourue (drone)</option>
      </select>
      <button @click="sortDir = -sortDir">
        {{ sortDir == 1 ? "▲" : "▼" }}
      </button>
    </p>
    <table v-for="mission in sortedMissions" :key="mission.id">
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
      <tr v-if="mission.map_rover">
        <th>Carte</th>
        <td>
          <img :src="`data:image/png;base64, ${mission.map_rover}`" />
        </td>
      </tr>
      <tr v-if="mission.map_drone">
        <th>Carte (drone)</th>
        <td>
          <img :src="`data:image/png;base64, ${mission.map_drone}`" />
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
table img {
  width: 200px;
  height: 200px;
  object-fit: contain;
  background-color: white;
}
.actions > button {
  margin-left: 10px;
}
</style>
