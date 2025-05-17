const express = require('express');
const cors = require('cors');
const rclnodejs = require('rclnodejs');
const path = require('path');
const { WebSocketServer } = require('ws');

const app = express();
const port = 3000;

// Enable CORS for all origins
app.use(cors());

let server;  // HTTP server reference
let wss;     // WebSocket server reference
let rosNode; // Global ROS 2 node reference

startRosConnection();

// const Fibonacci = rclnodejs.require('action_tutorials_interfaces/action/Fibonacci');

function startRosConnection() {

  rclnodejs.init().then(() => {
    
    rosNode = rclnodejs.createNode('ros2_interface_node');

    subscribeToTopic(rosNode, '/joint_states', 'sensor_msgs/msg/JointState');
    subscribeToTopic(rosNode, '/rosout', 'rcl_interfaces/msg/Log');

    app.get('/publish', (req, res) => {
      const topicName = req.query.topicName; // Get topic name from query parameter
      const messageType = req.query.messageType; // Get message type from query parameter
      const message = req.query.message || 'Hello from the web!';

      if (!topicName || !messageType) {
        res.status(400).send('Missing topicName or messageType query parameter.');
        return;
      }

      try {
        const publisher = rosNode.createPublisher(messageType, topicName);
        const responseMessage = publishToTopic(publisher, topicName, message);
        res.send(responseMessage);
      } catch (error) {
        res.status(500).send(error.message);
      }
    });


    app.get('/service', async (req, res) => {
      const serviceType = req.query.serviceType; // Get service type from query parameter
      const serviceName = req.query.serviceName; // Get service name from query parameter
      const request = req.query.request ? JSON.parse(req.query.request) : {}; // Parse request from query or use empty object

      if (!serviceType || !serviceName) {
        res.status(400).send('Missing serviceType or serviceName query parameter.');
        return;
      }

      try {
        const serviceClient = rosNode.createClient(serviceType, serviceName);
        const response = await callServiceDynamically(serviceName, serviceClient, request);
        res.send(`Service response: ${JSON.stringify(response)}`);
      } catch (error) {
        res.status(500).send(error.message);
      }
    });


    app.get('/action', async (req, res) => {
      const actionType = req.query.actionType; // Get action type from query parameter
      const actionName = req.query.actionName; // Get action name from query parameter
      const goalData = JSON.parse(req.query.goal) //  Parse goal from query parameter
      
      // const goalData = {
      //   "order": 10
      // };

      console.log(goalData);
    
      if (!actionType || !actionName) {
        res.status(400).send('Missing actionType or actionName query parameter.');
        return;
      }
    
      try {
        const _actionClient = new rclnodejs.ActionClient(rosNode, actionType, actionName);
    
        rosNode.getLogger().info('Waiting for action server...');
        await _actionClient.waitForServer();
            
        rosNode.getLogger().info('Sending goal request...');
        const goalHandle = await _actionClient.sendGoal(goalData, (feedback) =>
          feedbackCallback2(rosNode, feedback)
        );
    
        if (!goalHandle.isAccepted()) {
          rosNode.getLogger().info('Goal rejected');
          res.status(400).send('Goal rejected');
          return;
        }
    
        rosNode.getLogger().info('Goal accepted');
        const result = await goalHandle.getResult();
    
        if (goalHandle.isSucceeded()) {
          rosNode.getLogger().info(`Goal succeeded with result: ${result.sequence}`);
          res.send(`Goal succeeded with result: ${JSON.stringify(result.sequence)}`);
        } else {
          const status = goalHandle.getStatus();
          rosNode.getLogger().info(`Goal failed with status: ${status}`);
          res.status(500).send(`Goal failed with status: ${status}`);
        }
      } catch (error) {
        console.error('Error during action execution:', error);
        res.status(500).send(error.message);
      }
    });


    app.get('/parameter', async (req, res) => {
      const TargetNodeName = req.query.TargetNodeName; 
      const ParameterName = req.query.ParameterName;
      const ParameterType = req.query.ParameterType; 
      const ParameterValue = req.query.ParameterValue; 
      
      if (!TargetNodeName || !ParameterName || !ParameterType || !ParameterValue) {
        res.status(400).send('Missing TargetNodeName, ParameterName, ParameterType or ParameterValue query parameter.');
        return;
      }

      try {
        SetParameter(rosNode, TargetNodeName, ParameterName, ParameterType, ParameterValue);
      } catch (error) {
        res.status(500).send(error.message);
      }

    });

    rclnodejs.spin(rosNode);
  })
}


function feedbackCallback2(rosNode, feedback) {
  rosNode.getLogger().info(`Received feedback: ${feedback.sequence}`);
}


// Function to check ROS connection by listing publishers or services
function checkRosConnection(rosNode) {
  try {
    // Use a simple query like listing topics to verify connectivity
    const publishersInfo = rosNode.getTopicNamesAndTypes();
    
    // If no exception is thrown and we have topics, the node is connected
    return publishersInfo.length > 0;
  } catch (error) {
    console.error('Error while checking ROS connection:', error);
    return false;
  }
}


// Helper function to publish a message to a ROS 2 topic
function publishToTopic(publisher, topicName, message) {
  try { 
    publisher.publish(message);
    console.log(`Topic: ${topicName}, Published Message: ${message}`);
    return ``;
  } catch (error) {
    console.error(`Failed to publish message to topic '${topicName}':`, error);
    throw new Error(`Failed to publish message: ${error.message}`);
  }
}


// Call a ROS 2 service dynamically
async function callServiceDynamically(serviceName, serviceClient, request) {
  try {
    console.log(`Waiting for service '${serviceName}'...`);
    const isAvailable = await serviceClient.waitForService(1000);
    if (!isAvailable) {
      throw new Error(`Service '${serviceName}' is not available.`);
    }

    console.log(`Sending request to service '${serviceName}':`, request);
    return new Promise((resolve, reject) => {
      serviceClient.sendRequest(request, (response) => {
        resolve(response);
      });
    });
  } catch (error) {
    throw new Error(`Service call failed: ${error.message}`);
  }
}


// Set Parameter Value
async function SetParameter(rosNode, TargetNodeName, ParameterName, ParameterType, ParameterValue) {
  try {

    const client = rosNode.createClient(
      'rcl_interfaces/srv/SetParameters',
      `/${TargetNodeName}/set_parameters`
    );

    // Wait for the service to become available
    const serviceAvailable = await client.waitForService(1000);
    if (!serviceAvailable) {
      console.error(`Service not available: /${TargetNodeName}/set_parameters`);
      return;
    }

    // Determine the parameter value type and construct the request
    let value;
    switch (ParameterType) {
      case 'boolean':
        value = { type: rclnodejs.ParameterType.PARAMETER_BOOL, bool_value: ParameterValue };
        break;
      case 'integer':
        value = { type: rclnodejs.ParameterType.PARAMETER_INTEGER, integer_value: ParameterValue };
        break;
      case 'double':
        value = { type: rclnodejs.ParameterType.PARAMETER_DOUBLE, double_value: ParameterValue };
        break;
      case 'string':
        value = { type: rclnodejs.ParameterType.PARAMETER_STRING, string_value: ParameterValue };
        break;
      // Add other cases as needed
      default:
        console.error('Invalid parameter type specified.');
        return;
    }

    // Correct structure for SetParameters request
    const request = {
      parameters: [
        {
          name: ParameterName,
          value: value,
        },
      ],
    };

    console.log('Sending parameter update request:', request);

    // Correctly wrap sendRequest in a promise
    const response = await new Promise((resolve, reject) => {
      client.sendRequest(request, (response) => {
        if (response) {
          resolve(response);
        } else {
          reject(new Error('Failed to get a valid response from the service.'));
        }
      });
    });

    // Check the response results
    if (response.results && response.results[0] && response.results[0].successful) {
      console.log('Parameter updated successfully.');
    } else {
      console.error(
        'Failed to update parameter:',
        response.results[0] ? response.results[0].reason : 'Unknown error'
      );
    }
  } catch (error) {
    console.error('An error occurred while setting the parameter:', error);
  }
}


function setupWebSocket(server) {
  wss = new WebSocketServer({ server });

  wss.on('connection', (ws) => {
    console.log('WebSocket client connected.');
    ws.on('close', () => console.log('WebSocket client disconnected.'));
  });

  return wss;
}


// ROS Topic Subscriber for /joint_states
function subscribeToTopic(rosNode, topicName, messageType) {
  // console.log(`Subscribing to topic '${topicName}' with message type '${messageType}'...`);

  rosNode.createSubscription(messageType, topicName, (message) => {
    // console.log(`Received message on '${topicName}':`, message);

    // Format data for WebSocket clients
    const formattedData = {
      topicName,
      messageType,
      message,
    };

    // Send the message to WebSocket clients
    sendToWebSocketClients(formattedData);
  });
}


// Send data to all connected WebSocket clients
function sendToWebSocketClients(data) {
  if (wss && wss.clients) {
    wss.clients.forEach((client) => {
      if (client.readyState === client.OPEN) {
        client.send(JSON.stringify(data));
      }
    });
  }
}


// Serve the HTML file for the root route
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'index.html'));
});


// Example usage of the checkRosConnection function
app.get('/ros-status', (req, res) => {
  const isConnected = checkRosConnection(rosNode);
  // console.log(`ROS Connection Status: ${isConnected ? 'Connected' : 'Disconnected'}`);
  res.send({ isConnected });
});


// Example Node.js function to handle ROS disconnection
app.get('/disconnect-ros', (req, res) => {
  try {
    if (rosNode) {
      // Shutdown the ROS node to disconnect
      rclnodejs.shutdown(); // This will gracefully shutdown the ROS node
      rosNode = null; // Clear the ROS node reference
      res.status(200).json({ message: 'Disconnected from ROS successfully' });
    } else {
      res.status(400).send('No active ROS connection to disconnect from');
    }
  } catch (error) {
    res.status(500).send('Failed to disconnect from ROS: ' + error.message);
  }
});


// Start Express server
server = app.listen(port, () => {
  console.log(`Server running at http://localhost:${port}`);
  setupWebSocket(server); // Initialize WebSocket server
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
