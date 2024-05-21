ros2 pkg create client_pkg --build-type ament_cmake --node-name cliNode --dependencies std_msgs rclcpp



colcon build --packages-select client_pkg

source install/setup.bash


ros2 run client_pkg cliNode


