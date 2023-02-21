<script setup lang="ts">
import { inject } from "vue";
import { socketProvider } from "@/plugins/socket";
import type { Socket } from "socket.io-client";
import { ROBOT_STATUS } from "@/common/constants";

let roverStatus = ROBOT_STATUS.offline; 

const socket = inject(socketProvider) as Socket;

function start() {
  socket.emit("start", { data: "Démarrer mission" });
}

function identify() {
  socket.emit("identify", { data: "beep" });
}

function finish() {
  socket.emit("finish", { data: "Finir mission" });
}

socket.on("robotState", (in_mission) => {
  console.log("in_mission: " + in_mission)
  const robot_state = JSON.parse(in_mission);
  console.log("robot state" + robot_state);
  
  if (in_mission) {
    roverStatus = ROBOT_STATUS.in_mission;
  } else {
    roverStatus = ROBOT_STATUS.pending;
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
    <span>Le rover est {{ roverStatus }} </span>
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
</style>
