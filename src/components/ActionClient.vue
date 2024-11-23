<template>
    <div>
      <h1>Trigger ROS 2 Action</h1>
      <button @click="triggerAction">Trigger Action</button>
      <p v-if="responseMessage">{{ responseMessage }}</p>
    </div>
  </template>
  
  <script>
  export default {
    data() {
      return {
        responseMessage: null,
      };
    },
    methods: {
      async triggerAction() {
        try {
          const response = await fetch('http://localhost:3000/publish');
          const data = await response.json();
          this.responseMessage = `Result: ${data.success ? 'Success' : 'Failed'}. Message: ${data.message || data.result}`;
        } catch (error) {
          this.responseMessage = 'Error communicating with the server.';
          console.error('Error:', error);
        }
      },
    },
  };
  </script>
  
  <style scoped>
  /* Add any custom styles for the component */
  button {
    padding: 10px 20px;
    font-size: 16px;
  }
  </style>
  