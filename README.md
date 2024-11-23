# ros2_web_interface

## Project setup
```
npm install
```

### Compiles and hot-reloads for development
```
npm run serve
```

### Single ROS2 Action Client
```
add AppActionClient on main.js
node action_client.js
```

### ROS2 Publisher, Service Request, Action Client
```
add AppRosWorld on main.js
node ros_world.js
ros2 topic echo /my_topic
ros2 run action_tutorials_cpp fibonacci_action_server
python3 service_server.py 
```
