<script setup lang="ts">
import { RouterLink, RouterView } from "vue-router";
</script>

<template>
  <div class="container">
    <header>
      <ul class = "topPage">
        <span class = team>
          <img src = "./assets/cognifly.png" alt="logo">
        </span>
        <li><RouterLink to="/">Station</RouterLink></li>
        <li><RouterLink to="/missions">Historique</RouterLink></li>
        <li><RouterLink to="/logs">Logs</RouterLink></li> 
      </ul>
    </header>
  </div>
  
  <RouterView class="router-view" />
</template>
