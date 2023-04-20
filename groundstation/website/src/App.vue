<script lang="ts">
export default {
  data() {
    return {
      dropdownOpen: false,
    };
  },
  methods: {
    closeDropdown() {
      this.dropdownOpen = false;
    },
  },
};
</script>

<template>
  <div class="container">
    <header>
      <span class="team">
        <img src="./assets/fav.png" alt="logo" />
      </span>
      <div class="dropdown">
        <i
          class="fas fa-bars menu-icon"
          @click="dropdownOpen = !dropdownOpen"
        ></i>
        <ul class="dropdown-content" v-show="dropdownOpen">
          <li>
            <RouterLink to="/" @click="closeDropdown">Station</RouterLink>
          </li>
          <li>
            <RouterLink to="/missions" @click="closeDropdown"
              >Historique</RouterLink
            >
          </li>
          <li>
            <RouterLink to="/logs" @click="closeDropdown">Logs</RouterLink>
          </li>
        </ul>
        <ul class="topPage" v-show="!dropdownOpen">
          <li><RouterLink to="/">Station</RouterLink></li>
          <li><RouterLink to="/missions">Historique</RouterLink></li>
          <li><RouterLink to="/logs">Logs</RouterLink></li>
        </ul>
      </div>
    </header>
  </div>

  <RouterView class="router-view" />
</template>
