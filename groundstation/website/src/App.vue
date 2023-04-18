<script setup lang="ts">
import { RouterLink, RouterView } from "vue-router";
</script>

<template>
  <div class="container">
    <header>
      <ul>
        <span class = team>
          <img src = "./assets/cognifly.png" alt="logo">
        </span>
        <li><RouterLink to="/">Accueil</RouterLink></li>
        <li><RouterLink to="/missions">Missions</RouterLink></li>
        <li><RouterLink to="/logs">Logs</RouterLink></li> 
      </ul>
    </header>
  </div>
  
  <RouterView class="router-view" />
</template>
