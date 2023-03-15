<!-- eslint-disable vue/multi-word-component-names -->
<script lang="ts">
import { ref, inject, defineComponent } from "vue";
import { socketProvider } from "@/plugins/socket";
import type { Socket } from "socket.io-client";
import { ROBOT_STATUS } from "@/common/constants";

export default defineComponent({
  name: "Home",
  data() {
    return {
      rover: ref(ROBOT_STATUS.offline),
      drone: ref(ROBOT_STATUS.offline),
      socket: inject(socketProvider) as Socket,
      mapImageUrl: ref(""),
    };
  },
  methods: {
    start() {
      this.socket.emit("start", { data: "Démarrer mission" });
      console.log("start");
    },
    identify() {
      this.socket.emit("identify", { data: "beep" });
    },
    finish() {
      this.socket.emit("finish", { data: "Finir mission" });
    },
  },
  mounted() {
    this.socket.on("rover_state", (in_mission?: boolean) => {
      if (in_mission === true) {
        this.rover = ROBOT_STATUS.in_mission;
      } else if (in_mission === false) {
        this.rover = ROBOT_STATUS.pending;
      } else if (in_mission === undefined) {
        this.rover = ROBOT_STATUS.offline;
      }
    });

    this.socket.on("drone_state", (drone_in_mission?: boolean) => {
      if (drone_in_mission === true) {
        this.drone = ROBOT_STATUS.in_mission;
      } else if (drone_in_mission === false) {
        this.drone = ROBOT_STATUS.pending;
      } else if (drone_in_mission === undefined) {
        this.drone = ROBOT_STATUS.offline;
      }
    });

    this.socket.on("map_update", (map_bytes: ArrayBuffer) => {
      const arrayBufferView = new Uint8Array(map_bytes);
      const blob = new Blob([arrayBufferView], { type: 'image/png' });
      const imageUrl = URL.createObjectURL(blob);
      this.mapImageUrl = imageUrl;
      console.log(imageUrl);
    });
  },
});
</script>

<template>
  <div class="home">
    <h1 id="title">Gestion de la mission</h1>
    <div id="buttons">
      <button class="btn" @click="start">Lancer</button>
      <button class="btn" @click="identify">Identifier</button>
      <button class="btn" @click="finish">Terminer</button>
    </div>
    <div>
      <span class="robot_state">Le rover est {{ rover }} </span>
      <span class="robot_state">Le drone est {{ drone }} </span>
    </div>
  </div>
  <div class="image-container">
    <img class="image" :src="mapImageUrl" alt="Map" />
  </div>
</template>

<style scoped>
.home {
  display: flex;
  flex-direction: column;
  align-items: center;
  margin: 25px auto;
  justify-content: center;
  width: 80%;
  font-family: "Roboto", sans-serif;
  background: #df9d81;
  padding: 25px;
}
.image-container{
  display: flex;
  justify-content: center;
}
.image{
  height: 50vh;
}
#title {
  color: #943e36;
  font-weight: bold;
  font-size: 50px;
  text-align: center;
}
#buttons {
  display: flex;
  gap: 25px;
}
.btn {
  background-color: #13ce66;
  font-size: 20px;
  font-family: "Roboto", sans-serif;
  padding: 10px;
  color: rgb(55, 21, 21);
  cursor: pointer;
}
.btn:hover {
  background-color: #0e9f4f;
}
.robot_state {
  margin-top: 10px;
  font-size: 20px;
  background-color: #d9c1b9;
  color: #604d44;
  padding: 10px;
  border-radius: 5px;
  box-shadow: 0px 0px 10px rgba(0, 0, 0, 0.2);
  width: 230px;
  text-align: center;
  position: relative;
  overflow: hidden;
}

span {
  display: block;
}
</style>
