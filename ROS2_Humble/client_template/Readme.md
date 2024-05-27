

# 服务的客户端模板

服务名称绑定为"add_two_ints", 功能是接收客户端发送过来的两个int参数,然后将这个两个变量相加,返回相加结果给客户端


- 创建指令:




ros2 pkg create client_pkg --build-type ament_cmake --node-name cliNode --dependencies std_msgs rclcpp



colcon build --packages-select client_pkg

source install/setup.bash


ros2 run client_pkg cliNode


