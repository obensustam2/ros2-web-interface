<template>
    <div id="app">
      <!-- Add the ActionClient component here -->
      <ActionClient />
    </div>
  </template>
  
  <script>
  // Import the ActionClient.vue component
  import ActionClient from './components/ActionClient.vue';
  
  export default {
    name: 'App',
    components: {
      ActionClient, // Register it here
    },
  };
  </script>
  
  <style>
  /* Add global styles if needed */
  </style>
  