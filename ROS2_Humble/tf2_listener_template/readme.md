



ros2 pkg create listener_pkg --build-type ament_cmake --node-name listenerNode --dependencies geometry_msgs rclcpp tf2 tf2_ros turtlesim



colcon build --packages-select listener_pkg listenerNode




source install/setup.bash

ros2 run listener_pkg listenerNode









