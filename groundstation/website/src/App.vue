<script setup lang="ts">
import { RouterLink, RouterView } from "vue-router";
</script>

<template>
  <header>
    <div class="wrapper">
      <nav class="toolbar">
        <RouterLink to="/">Accueil</RouterLink>
        <RouterLink to="/missions">Missions</RouterLink>
        <RouterLink to="/logs">Logs</RouterLink>
      </nav>
    </div>
  </header>

  <RouterView class="router-view" />
</template>
