

ros2 pkg create act_cli_pkg --build-type ament_cmake --node-name actCliNode --dependencies std_msgs rclcpp



colcon build --packages-select action_pkg act_cli_pkg

source install/setup.bash


ros2 run act_cli_pkg actCliNode