
ROS2的参数服务器是一个用于存储节点参数的分布式键值对存储系统。它允许节点在运行时动态地获取和设置参数，而不需要重新编译或重新启动节点。参数服务器可以在本地节点上存储参数，也可以是一个远程的参数服务

示例中设置参数名为"myparameter",初始值为1,然后循环回调函数中,循环的将这个变量自增


创建包:

ros2 pkg create param_pkg --build-type ament_cmake --node-name paramNode --dependencies std_msgs rclcpp

colcon build --packages-select param_pkg


source install/setup.bash


ros2 run param_pkg paramNode


打印结果

wub@wubr:~/code/ros2_humble/param_template$ ros2 run param_pkg paramNode
[INFO] [1716367110.972655282] [minimal_param_node]: Hello 1!
[INFO] [1716367111.971896768] [minimal_param_node]: Hello 2!
[INFO] [1716367112.972388470] [minimal_param_node]: Hello 3!
[INFO] [1716367113.972020327] [minimal_param_node]: Hello 4!







