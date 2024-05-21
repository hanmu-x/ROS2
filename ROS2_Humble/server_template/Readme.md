- 创建指令:

ros2 pkg create server_pkg --build-type ament_cmake --node-name svrNode --dependencies std_msgs rclcpp


colcon build --packages-select server_pkg

source install/setup.bash


ros2 run server_pkg svrNode

