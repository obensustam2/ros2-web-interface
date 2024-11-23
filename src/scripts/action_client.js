'use strict';

const express = require('express');
const rclnodejs = require('rclnodejs');
const path = require('path');

const Fibonacci = rclnodejs.require('action_tutorials_interfaces/action/Fibonacci');

const app = express();
const port = 3000;

// CORS setup (to allow communication from the frontend)
const cors = require('cors');
app.use(cors());

// Serve the Vue.js app when accessing the root route
app.use(express.static(path.join(__dirname, 'dist')));

// ROS 2 Action Client Class
class FibonacciActionClient {
  constructor(node) {
    this._node = node;
    this._actionClient = new rclnodejs.ActionClient(
      node,
      'action_tutorials_interfaces/action/Fibonacci',
      'fibonacci'
    );
  }

  async sendGoal() {
    try {
      this._node.getLogger().info('Waiting for action server...');
      await this._actionClient.waitForServer(5000); // Wait with a timeout

      const goal = new Fibonacci.Goal();
      goal.order = 10;

      this._node.getLogger().info('Sending goal request...');
      const goalHandle = await this._actionClient.sendGoal(goal, (feedback) =>
        this.feedbackCallback(feedback)
      );

      if (!goalHandle.isAccepted()) {
        this._node.getLogger().info('Goal rejected');
        return { success: false, message: 'Goal rejected' };
      }

      this._node.getLogger().info('Goal accepted');
      const result = await goalHandle.getResult();

      if (goalHandle.isSucceeded()) {
        this._node.getLogger().info(`Goal succeeded with result: ${result.sequence}`);
        return { success: true, result: result.sequence };
      } else {
        this._node.getLogger().info(`Goal failed with status: ${goalHandle.getGoalStatus()}`);
        return { success: false, message: `Goal failed: ${goalHandle.getGoalStatus()}` };
      }
    } catch (error) {
      this._node.getLogger().error(`Error: ${error.message}`);
      throw error;
    }
  }

  feedbackCallback(feedback) {
    this._node.getLogger().info(`Received feedback: ${feedback.sequence}`);
  }
}

rclnodejs.init().then(() => {
    const node = rclnodejs.createNode('action_client_example_node');
    const client = new FibonacciActionClient(node);

    // Define endpoint to trigger the action client
    app.get('/publish', async (req, res) => {
      try {
        const result = await client.sendGoal();
        res.json(result);
      } catch (error) {
        res.status(500).send('Failed to execute action');
        console.error(error);
      }
    });

    rclnodejs.spin(node);
  })
  .catch((err) => {
    console.error(err);
  });

// Start the Express server
app.listen(port, () => {
  console.log(`Server running at http://localhost:${port}`);
});
