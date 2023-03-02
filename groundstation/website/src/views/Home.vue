<script setup lang="ts">
import { ref, inject } from "vue";
import { socketProvider } from "@/plugins/socket";
import type { Socket } from "socket.io-client";
import { ROBOT_STATUS } from "@/common/constants";


let socket = inject(socketProvider) as Socket;
const rover = ref(ROBOT_STATUS.offline);
const drone = ref(ROBOT_STATUS.offline);

function start() {
  socket.emit("start", { data: "Démarrer mission" });
  console.log("start");
}

function identify() {
  socket.emit("identify", { data: "beep" });
}

function finish() {
  socket.emit("finish", { data: "Finir mission" });
}

socket.on("rover_state", (in_mission) => {
  if (in_mission === true) {
    rover.value = ROBOT_STATUS.in_mission;
  } else if (in_mission === false) {
    rover.value = ROBOT_STATUS.pending;
  } else if (in_mission === undefined) {
    rover.value = ROBOT_STATUS.offline;
  }
});

socket.on("drone_state", (drone_in_mission) => {
  if (drone_in_mission === true) {
    drone.value = ROBOT_STATUS.in_mission;
  } else if (drone_in_mission === false) {
    drone.value = ROBOT_STATUS.pending;
  } else if (drone_in_mission === undefined) {
    drone.value = ROBOT_STATUS.offline;
  }
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
