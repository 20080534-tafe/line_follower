# Wall Avoider ROS2 Package

This package implements a ROS2-based wall avoiding controller for the Webots e-puck robot.

## Features
- Subscribes to distance sensor values from Webots (/ps7)
- Publishes /cmd_vel velocity commands
- Uses standard ROS2 message types (geometry_msgs/Twist) (geometry_msgs/TwistStamped)
- Parameters stored in YAML file
- Modular and hardware-agnostic architecture
- Stops and turns 90 degrees when obstacles are detected within 0.05 meters

## Launching
Build workspace:

    cd ~/ros2_ws
    colcon build
    source install/setup.bash

Run simulation:

    ros2 launch line_follower line_follower_launch.py

## ROS2 Pre-existing Components Used

- **ROS2 Middleware (`rclpy`)** – for Python-based ROS2 node development
- **`webots_ros2_driver`** – to interface Webots sensors and actuators with ROS2
- **Standard ROS2 messages** – `geometry_msgs/Twist` for velocity control, `sensor_msgs` for distance sensors
- **ROS2 Parameter System** – to configure robot speed and sensor thresholds externally
- **ROS2 Launch System** – to start Webots and the ROS2 node together
