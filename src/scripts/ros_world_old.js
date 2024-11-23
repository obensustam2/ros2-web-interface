const express = require('express');
const cors = require('cors');  // Import the CORS middleware
const rclnodejs = require('rclnodejs');
const path = require('path');

const MyService = rclnodejs.require('std_srvs/srv/Empty');  // Replace with the correct package name
const Fibonacci = rclnodejs.require('action_tutorials_interfaces/action/Fibonacci');

const app = express();
const port = 3000;

// Enable CORS for all origins
app.use(cors());

// Serve the HTML file for the root route
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'index.html'));
});

// ROS 2 Publisher (Topic)
rclnodejs.init().then(() => {
  const node = rclnodejs.createNode('ros2_interface_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'my_topic');

  // Publish message to topic
  app.get('/publish', (req, res) => {
    const message = req.query.message || 'Hello from the web!';
    publisher.publish(message);
    console.log(`Published message: ${message}`);
    res.send(`Message published: ${message}`);
  });

  // Service Request (Sum)
  const serviceClient = node.createClient(MyService, 'start');  // Replace with actual service name

  // Endpoint to send service request
  app.get('/service', async (req, res) => {
    try {
      const request = {}; // Empty request object
    
      let result = await serviceClient.waitForService(1000);
      if (!result) {
        console.log('Error: service not available');
        rclnodejs.shutdown();
        return;
      }
    
      console.log('Sending empty service request');
      serviceClient.sendRequest(request, (response) => {
        console.log('Result received:', response);
        rclnodejs.shutdown();
      });
    
    } catch (error) {
      console.error('Error calling service:', error);
      res.status(500).send('Service request failed');
    }
    
  });

  // Action Goal (Fibonacci)
  const actionClient = new rclnodejs.ActionClient(node,'action_tutorials_interfaces/action/Fibonacci','fibonacci');

  app.get('/action', async (req, res) => {
    try {
      const goal = new Fibonacci.Goal();
      goal.order = 10;

      await actionClient.waitForServer();
      const goalHandle = await actionClient.sendGoal(goal, (feedback) => {
        console.log('Received feedback:', feedback.sequence);
      });

      if (!goalHandle.isAccepted()) {
        res.status(400).send('Goal was rejected');
        return;
      }

      console.log('Goal accepted, waiting for result...');
      const result = await goalHandle.getResult();
      res.send(`Fibonacci goal succeeded with result: ${result.sequence}`);
    } catch (error) {
      console.error('Error sending action goal:', error);
      res.status(500).send('Action goal failed');
    }
  });

  rclnodejs.spin(node);
}).catch((err) => {
  console.error('Error initializing ROS 2:', err);
});

// Graceful Shutdown
process.on('SIGINT', () => {
  console.log('\nShutting down gracefully...');
  if (rosNode) {
    rclnodejs.shutdown(); // Properly shut down rclnodejs
    console.log('ROS 2 node stopped.');
  }
  process.exit(0); // Terminate the Node.js process
});

app.listen(port, () => {
  console.log(`Server running at http://localhost:${port}`);
});
