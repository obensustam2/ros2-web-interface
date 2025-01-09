<template>
  <RosWorld></RosWorld>
</template>
  
<script>
  import RosWorld from './components/RosWorld.vue'
  
  export default {
    name: 'App',
    components: {
        RosWorld
    }
  }
</script>
  
<style>
  #app {
    font-family: Avenir, Helvetica, Arial, sans-serif;
    -webkit-font-smoothing: antialiased;
    -moz-osx-font-smoothing: grayscale;
    text-align: center;
    color: #2c3e50;
    margin-top: 60px;
  }
</style>