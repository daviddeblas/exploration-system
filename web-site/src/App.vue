<template>
  <body>
    <div class="toolbar">
      <a class="active" href="#home">INF3995 - 101 </a>
      <a id="tools" href="#about"></a>
    </div>
    <Home class= 'container'/>
  </body>
  
</template>

<script lang="ts">
import { Options, Vue } from 'vue-class-component';
import Home from './components/Home.vue';

@Options({
  components: {
    Home,
  },
})
export default class App extends Vue {}
</script>

<style>
#app {
  margin-top: 0;
  font-family: Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  color: #2c3e50;
}

body {
  background-image: url("assets/background.jpg");
}

.container {
  margin-top : 5px;
  text-align: center;
}

.toolbar {
  overflow: hidden;
  background-color: #4f312b;
  font-family: 'Roboto', sans-serif;
  margin-left: 10px;
  font-weight: bold;
}

.toolbar a {
  float: left;
  color: #f2f2f2;
  text-align: center;
  padding: 14px 16px;
  text-decoration: none;
  font-size: 17px;
}

.toolbar a:hover {
  background-color: #ddd;
  color: black;
}

.toolbar a.active {
  background-color: #a24036;
  color: white;
}
</style>
