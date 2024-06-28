

# 创建action包

ros2 pkg create action_pkg --build-type ament_cmake --node-name actNode --dependencies std_msgs rclcpp

# 创建服务包

ros2 pkg create action_proj --build-type ament_cmake --node-name actProjNode --dependencies std_msgs rclcpp

colcon build --packages-select action_pkg action_proj

source install/setup.bash


ros2 run action_proj actProjNode



wub@wubr:~/code/ros2_humble/action_server_template$ ros2 run action_proj actProjNode 
[INFO] [1719539225.199796318] [rotate_action_server]: Received goal request: 270 degrees
[INFO] [1719539225.200180875] [rotate_action_server]: Target angle: 270
[INFO] [1719539227.001015910] [rotate_action_server]: Server Goal succeeded, final angle: 270 degrees






