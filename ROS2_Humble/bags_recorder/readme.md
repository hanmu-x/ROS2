
用于记录话题的发布,生成一个ros2的包

示例中订阅了两个话题,一个话题是 std_msgs/msg/String ros2自带的字符串类型
另一个是publish_msg/msg/MyCustomMsg 自定义的数据类型包

## 创建包

ros2 pkg create bags_recorder_pkg --build-type ament_cmake --node-name bagsRecorderNode --dependencies example_interfaces rclcpp rosbag2_cpp std_msgs



source install/setup.bash

ros2 run bags_recorder_pkg bagsRecorderNode

会在src目录中生产一个my_bag的目录
播放记录的包
ros2 bag play my_bag/

验证 ros2 topic list 查看是否出现两个订阅的话题






