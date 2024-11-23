<template>
    <div id="app">
      <!-- Add the RosWorld component here -->
      <RosWorld />
    </div>
  </template>
  
  <script>
  // Import the RosWorld.vue component
  import RosWorld from './components/RosWorld.vue';
  
  export default {
    name: 'App',
    components: {
      RosWorld, // Register it here
    },
  };
  </script>
  
  <style>
  /* Add global styles if needed */
  </style>
  