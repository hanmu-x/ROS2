

# 创建action包

ros2 pkg create action_pkg --build-type ament_cmake --node-name actNode --dependencies std_msgs rclcpp

# 创建服务包

ros2 pkg create action_proj --build-type ament_cmake --node-name actProjNode --dependencies std_msgs rclcpp

colcon build --packages-select action_pkg action_proj

source install/setup.bash


ros2 run action_proj actProjNode









