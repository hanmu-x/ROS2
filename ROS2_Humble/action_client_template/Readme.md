

ros2 pkg create act_cli_pkg --build-type ament_cmake --node-name actCliNode --dependencies std_msgs rclcpp



colcon build --packages-select action_pkg act_cli_pkg

source install/setup.bash


ros2 run act_cli_pkg actCliNode




实现了一个 ROS 2 动作客户端，用于向名为 "rotate" 的动作服务器发送旋转目标，并处理服务器的响应、反馈和最终结果











