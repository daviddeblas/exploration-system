<script setup lang="ts">
import { inject } from "vue";
import { socketProvider } from "@/plugins/socket";
import type { Socket } from "socket.io-client";

const socket = inject(socketProvider) as Socket;

function indentify() {
  socket.emit("beep", { data: "beep" });
}
</script>

<template>
  <div class="home">
    <h1 id="title">Gestion de la mission</h1>
    <div id="buttons">
      <button class="btn">Lancer</button>
      <button class="btn" @click="indentify">Identifier</button>
      <button class="btn">Terminer</button>
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
