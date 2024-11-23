<template>
  <div>
    <h1>ROS 2 Web Interface</h1>

    <!-- Buttons with spacing -->
    <div class="button-container">
      <button @click="publishTopic">Publish to Topic</button>
      <button @click="sendServiceRequest">Send Service Request</button>
      <button @click="sendActionGoal">Send Action Goal</button>
    </div>
  </div>
</template>

<script>
export default {
  methods: {
    async publishTopic() {
      try {
        const response = await fetch('http://localhost:3000/publish?message=Hello from the button!');
        if (!response.ok) {
          throw new Error('Failed to publish message');
        }
        // const text = await response.text();
        // alert(text);
      } catch (error) {
        alert(`Error: ${error.message}`);
      }
    },

    async sendServiceRequest() {
      try {
        const response = await fetch('http://localhost:3000/service');
        if (!response.ok) {
          throw new Error('Failed to send service request');
        }
        // const text = await response.text();
        // alert(text);
      } catch (error) {
        alert(`Error: ${error.message}`);
      }
    },

    async sendActionGoal() {
      try {
        const response = await fetch('http://localhost:3000/action?goal=StartFibonacci');
        if (!response.ok) {
          throw new Error('Failed to send action goal');
        }
        // const text = await response.text();
        // alert(text);
      } catch (error) {
        alert(`Error: ${error.message}`);
      }
    }
  }
}
</script>

<style scoped>
/* Style for button spacing */
.button-container {
  display: left;
  flex-direction: column; /* Arrange buttons vertically */
  gap: 20px; /* Add spacing between buttons */
}

button {
  padding: 10px 20px;
  font-size: 16px;
  cursor: pointer;
}
</style>
