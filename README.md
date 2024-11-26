# ROS2 Web Interface

## Project setup
```
cd ros2-web-interface/
npm install
```

## Project launch 
```
cd ros2-web-interface/
npm run serve
source <ros2_ws>/install/setup.bash
node src/ros_world.js
ros2 topic echo /my_topic
python3 src/ros-scripts/fibonacci_action_server.py
python3 src/ros-scripts/service_server.py
python3 src/ros-scripts/get_param.py
ws
ros2 launch fanuc_bringup fanuc_bringup.launch.py
```

## ToDo
- Adding default json data for the Action Goal and Service Request

