

ros2 pkg create act_cli_pkg --build-type ament_cmake --node-name actCliNode --dependencies std_msgs rclcpp



colcon build --packages-select action_pkg act_cli_pkg

source install/setup.bash


ros2 run act_cli_pkg actCliNode




实现了一个 ROS 2 动作客户端，用于向名为 "rotate" 的动作服务器发送旋转目标，并处理服务器的响应、反馈和最终结果





wub@wubr:~/code/ros2_humble/action_client_template$ ros2 run act_cli_pkg actCliNode 
[INFO] [1719539623.646120815] [rotate_action_client]: Goal accepted by server, waiting for result
[INFO] [1719539623.656792207] [rotate_action_client]: Current rotation angle: 2 degrees
[INFO] [1719539623.666813195] [rotate_action_client]: Current rotation angle: 3 degrees
[INFO] [1719539623.676714096] [rotate_action_client]: Current rotation angle: 4 degrees
............................
[INFO] [1719539625.416923155] [rotate_action_client]: Current rotation angle: 268 degrees
[INFO] [1719539625.426430604] [rotate_action_client]: Current rotation angle: 269 degrees
[INFO] [1719539625.436567251] [rotate_action_client]: Current rotation angle: 270 degrees
[INFO] [1719539625.446909976] [rotate_action_client]: client Goal succeeded with final angle: 270 degrees
^C[INFO] [1719539628.138842399] [rclcpp]: signal_handler(signum=2)






