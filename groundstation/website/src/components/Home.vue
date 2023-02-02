<template>
  <div class="home">
    <p id="title">
      Gestion de la mission
    </p>
    <div id="button"> 
      <button class="btn" id="left" > Lancer </button>
      <button class="btn" id="right"> Terminer </button>
    </div>
    
  </div>
</template>

<script lang="ts">
import { Options, Vue } from 'vue-class-component';

@Options({
  props: {
    msg: String
  }
})
export default class Main extends Vue {
  msg!: string
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
h3 {
  margin: 40px 0 0;
}
ul {
  list-style-type: none;
  padding: 0;
}
li {
  display: inline-block;
  margin: 0 10px;
}
a {
  color: #42b983;
}
.home {
    display: flex;
    flex-direction: column;
    align-content: center;
    align-items: center;
    justify-content:center;
    width: 80%;
    height: 40%;
    margin-left: 10.5%;
    font-family: 'Roboto', sans-serif;
    line-height: normal;
    position: absolute;
    top: 50%;
    transform: translateY(-50%);
    font-size: 60px;
    text-align: center;
    background: #df9d81;
    padding-top: 2%;
    padding-bottom: 2%;
  }
  #title {
    color: #943e36;
    font-family: 'Roboto', sans-serif;
    margin-left: 10px;
    font-weight: bold;
  }
  .btn {
    background-color: #13ce66;
    font-size: 20px;
    font-family: 'Roboto', sans-serif;
    padding: 10px;
    color: rgb(55, 21, 21);
    cursor: pointer;
  }
  #right {
    position: absolute; 
    right: 35%; 
  }
  #left {
    position: absolute; 
    left: 35%; 
  }

</style>
