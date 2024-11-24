<template>
  <div>
    <h1>ROS 2 Web Interface</h1>

    <!-- ROS Connection Status Section -->
    <!-- ROS Connection Status Section -->
    <div class="section">
      <h2>ROS Connection Status</h2>
      <p v-if="rosStatus !== null">
        ROS Connection: <strong>{{ rosStatus ? 'Connected' : 'Disconnected' }}</strong>
      </p>
      <div class="button-container">
        <button @click="connectRos" class="blue-button">Connect ROS</button>
        <button @click="disconnectRos" class="blue-button">Disconnect ROS</button>
      </div>
    </div>

    <div class="section">
      <h2>Received Joint States</h2>
      <div v-if="jointStates.length > 0">
        <!-- <p><strong>Topic:</strong> {{ jointStates[0].topicName }}</p>
        <p><strong>Message Type:</strong> {{ jointStates[0].messageType }}</p> -->
        <p><strong>Joint Names:</strong> {{ jointStates[0].message.name }}</p>
        <p><strong>Positions:</strong> {{ jointStates[0].message.position }}</p>
        <!-- <p><strong>Velocities:</strong> {{ jointStates[0].message.velocity }}</p>
        <p><strong>Efforts:</strong> {{ jointStates[0].message.effort }}</p> -->
      </div>
      <div v-else>
        <p>No joint states received yet.</p>
      </div>
    </div>


    <div class="section">
      <h2>Latest Log Messages</h2>
      <div v-for="message in rosoutMessages" :key="message.message.level" class="message">
        <p><strong>{{ formatLevel(message.message.level) }}:</strong> {{ message.message.msg }}</p>
      </div>
    </div>

    <!-- Other Sections -->
    <div class="sections-row">

      <!-- Set Parameter Section -->
      <div class="section">
        <h2>Send Parameter Update</h2>
        <input v-model="parameter.TargetNodeName" placeholder="Target Node Name" />
        <input v-model="parameter.ParameterName" placeholder="Parameter Name" />
        <input v-model="parameter.ParameterType" placeholder="Parameter Type" />
        <input v-model="parameter.ParameterValue" placeholder="Parameter Value" />
        <button @click="sendParameterUpdate" class="blue-button">Update Parameter</button>
      </div>

      <!-- Publish to Topic Section -->
      <div class="section">
        <h2>Publish to Topic</h2>
        <input v-model="publish.topicName" placeholder="Topic Name (default: my_topic)" />
        <input v-model="publish.messageType" placeholder="Message Type" />
        <input v-model="publish.message" placeholder="Message" />
        <button @click="publishTopic" class="blue-button">Publish</button>
      </div>

      <!-- Send Service Request Section -->
      <div class="section">
        <h2>Send Service Request</h2>
        <input v-model="service.serviceName" placeholder="Service Name" />
        <input v-model="service.serviceType" placeholder="Service Type" />
        <textarea v-model="service.request" placeholder="Request (JSON format)"></textarea>
        <button @click="sendServiceRequest" class="blue-button">Send Service Request</button>
      </div>

      <!-- Send Action Goal Section -->
      <div class="section">
        <h2>Send Action Goal</h2>
        <input v-model="action.actionName" placeholder="Action Name" />
        <input v-model="action.actionType" placeholder="Action Type" />
        <textarea v-model="action.goal" placeholder="Goal (JSON format)"></textarea>
        <button @click="sendActionGoal" class="blue-button">Send Action Goal</button>
      </div>
    </div>

  </div>

</template>

<script>
export default {
  data() {
    return {
      jointStates: [], // Array to store received joint states
      rosoutMessages: [],
      rosStatus: null,
      isRosStatusPressed: false,
      parameter: {
        TargetNodeName: 'fanuc_gripper_multi_group',
        ParameterName: 'max_velocity_scaling_factor',
        ParameterType: 'double',
        ParameterValue: 0.5,
      },
      publish: {
        topicName: 'my_topic',
        messageType: 'std_msgs/msg/String',
        message: 'hello'
      },
      service: {
        serviceName: 'start',
        serviceType: 'std_srvs/srv/Empty',
        request: ''
      },
      action: {
        actionName: 'fibonacci',
        actionType: 'action_tutorials_interfaces/action/Fibonacci',
        goal: ''
      }
    };
  },


  created() {
    // Periodically check ROS status
    this.checkRosStatus();
    this.statusInterval = setInterval(this.checkRosStatus, 1000);

    // Connect to the WebSocket server
    const socket = new WebSocket('ws://localhost:3000');

    socket.onmessage = (event) => {
      const data = JSON.parse(event.data);

      // Only handle messages from /joint_states
      if (data.topicName === '/joint_states') {
        console.log('Received /joint_states message:', data);

        // Replace the jointStates array with the latest message
        this.jointStates = [{id: Date.now(), ...data,}]; // Unique ID for each message
      }
    };

    socket.onopen = () => {
      console.log('WebSocket connected.');
    };

    socket.onclose = () => {
      console.log('WebSocket disconnected.');
    };

    this.socket = socket; // Store the socket reference for future use
  },


  beforeUnmount() {
    // Clear interval when the component is destroyed
    clearInterval(this.statusInterval);
  },

  methods: {
    async checkRosStatus() {
      try {
        const response = await fetch('http://localhost:3000/ros-status');
        const data = await response.json();
        this.rosStatus = data.isConnected;
      } catch (error) {
        console.error(`Error checking ROS connection: ${error.message}`);
        this.rosStatus = false; // Assume disconnected on error
      }
    },


    async connectRos() {
      try {
        await fetch('http://localhost:3000/connect-ros');
      } catch (error) {
        alert(`Error connecting to ROS: ${error.message}`);
      }
    },


    async disconnectRos() {
      try {
        await fetch('http://localhost:3000/disconnect-ros');
      } catch (error) {
        alert(`Error disconnecting from ROS: ${error.message}`);
      }
    },


    async publishTopic() {
      const { topicName, messageType, message } = this.publish;
      if (!topicName || !messageType || !message) {
        alert('Please provide Topic Name, Message Type, and Message.');
        return;
      }
      try {
        const response = await fetch(
          `http://localhost:3000/publish?topicName=${topicName}&messageType=${messageType}&message=${encodeURIComponent(
            message
          )}`
        );
        if (!response.ok) {
          throw new Error('Failed to publish message');
        }
      } catch (error) {
        alert(`Error: ${error.message}`);
      }
    },


    async sendServiceRequest() {
      const { serviceType, serviceName, request } = this.service;
      if (!serviceType || !serviceName) {
        alert('Please provide Service Type and Service Name.');
        return;
      }
      try {
        const response = await fetch(
          `http://localhost:3000/service?serviceType=${serviceType}&serviceName=${serviceName}&request=${encodeURIComponent(
            request
          )}`
        );
        if (!response.ok) {
          throw new Error('Failed to send service request');
        }
      } catch (error) {
        alert(`Error: ${error.message}`);
      }
    },


    async sendActionGoal() {
      const { actionType, actionName, goal } = this.action;
      if (!actionType || !actionName) {
        alert('Please provide Action Type and Action Name.');
        return;
      }
      try {
        const response = await fetch(
          `http://localhost:3000/action?actionType=${actionType}&actionName=${actionName}&goal=${encodeURIComponent(
            goal
          )}`
        );
        if (!response.ok) {
          throw new Error('Failed to send action goal');
        }
      } catch (error) {
        alert(`Error: ${error.message}`);
      }
    },


    async sendParameterUpdate() {
      const { TargetNodeName, ParameterName, ParameterType, ParameterValue, request} = this.parameter;
      if (!TargetNodeName || !ParameterName || !ParameterType || !ParameterValue) {
        alert('Please provide required parameter data');
        return;
      }
      try {
        const response = await fetch(
          `http://localhost:3000/parameter?TargetNodeName=${TargetNodeName}&ParameterName=${ParameterName}&ParameterType=${ParameterType}&ParameterValue=${ParameterValue}&request=${encodeURIComponent(
            request
          )}`
        );
        if (!response.ok) {
          throw new Error('Failed to set parameter request');
        }
      } catch (error) {
        alert(`Error: ${error.message}`);
      }
    }
  }
};
</script>

<style scoped>
  .sections-row {
    display: flex;
    justify-content: space-between;  /* Distribute space evenly */
    gap: 20px;  /* Space between the sections */
  }

  .section {
    flex: 1;  /* Each section takes equal space */
    padding: 10px;
    background-color: #f9f9f9;
    border-radius: 5px;
  }

  .section h2 {
    text-align: center;
  }

  input, textarea {
    width: 100%;
    margin-bottom: 10px;
    padding: 8px;
    border: 1px solid #ccc;
    border-radius: 4px;
  }

  button {
    width: 100%;
    padding: 10px;
    border: none;
    border-radius: 5px;
    background-color: #007BFF;
    color: white;
    cursor: pointer;
  }

  button:hover {
    background-color: #0056b3;
  }


  h2 {
    background-color: lightcoral;
    color: white;
    padding: 3px;
    border-radius: 3px; /* Optional: adds rounded corners */
  }

  textarea {
    height: 100px;
    resize: none;
  }

 .button-container {
    display: flex;
    gap: 10px; /* Optional: Adjust space between buttons */
    justify-content: center; /* Optional: Center align buttons horizontally */
  }

  .blue-button {
    padding: 10px 20px;
    background-color: #799fc8;
    color: white;
    border: none;
    border-radius: 5px;
    cursor: pointer;
  }

  .blue-button:hover {
    background-color: #0056b3; /* Darker blue on hover */
  }


  .blue-button:active {
    background-color: #555;
  }

  button.pressed {
    background-color: #666;
  }

  p {
    font-size: 18px;
    margin: 10px 0;
  }

  .message {
    padding: 10px;
    margin-bottom: 10px;
    border: 1px solid #ccc;
    border-radius: 5px;
  }
</style>
