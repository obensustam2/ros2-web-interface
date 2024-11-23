const express = require('express');
const cors = require('cors');
const rclnodejs = require('rclnodejs');
const path = require('path');

const MyService = rclnodejs.require('std_srvs/srv/Empty');
const Fibonacci = rclnodejs.require('action_tutorials_interfaces/action/Fibonacci');

const app = express();
const port = 3000;

// Enable CORS for all origins
app.use(cors());

let rosNode; // Global ROS 2 node reference
let server;  // HTTP server reference

// Serve the HTML file for the root route
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'index.html'));
});

// ROS 2 Initialization
rclnodejs.init().then(() => {
  rosNode = rclnodejs.createNode('ros2_interface_node');
  const publisher = rosNode.createPublisher('std_msgs/msg/String', 'my_topic');

  // Publish message to topic
  app.get('/publish', (req, res) => {
    const message = req.query.message || 'Hello from the web!';
    publisher.publish(message);
    console.log(`Published message: ${message}`);
    res.send(`Message published: ${message}`);
  });

  // Service Request
  const serviceClient = rosNode.createClient(MyService, 'start');

  app.get('/service', async (req, res) => {
    try {
      const request = {}; // Empty request object
      const result = await serviceClient.waitForService(1000);

      if (!result) {
        console.log('Error: service not available');
        res.status(503).send('Service not available');
        return;
      }

      console.log('Sending empty service request...');
      serviceClient.sendRequest(request, (response) => {
        console.log('Result received:', response);
        res.send(`Service response: ${JSON.stringify(response)}`);
      });
    } catch (error) {
      console.error('Error calling service:', error);
      res.status(500).send('Service request failed');
    }
  });

  // Action Goal
  const actionClient = new rclnodejs.ActionClient(rosNode, 
                                                  'action_tutorials_interfaces/action/Fibonacci',
                                                  'fibonacci');

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
      res.send(`Fibonacci goal succeeded with result: ${result.partial_sequence}`);
    } catch (error) {
      console.error('Error sending action goal:', error);
      res.status(500).send('Action goal failed');
    }
  });

  rclnodejs.spin(rosNode);
}).catch((err) => {
  console.error('Error initializing ROS 2:', err);
});

// Start Express server
server = app.listen(port, () => {
  console.log(`Server running at http://localhost:${port}`);
});

// Graceful Shutdown
process.on('SIGINT', async () => {
  console.log('\nShutting down gracefully...');
  try {
    if (rosNode) {
      await rclnodejs.shutdown();
      console.log('ROS 2 node stopped.');
    }
    if (server) {
      server.close(() => {
        console.log('HTTP server closed.');
      });
    }
  } catch (error) {
    console.error('Error during shutdown:', error);
  } finally {
    process.exit(0);
  }
});
