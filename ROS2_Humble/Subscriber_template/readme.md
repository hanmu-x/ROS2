
这是一个ros2订阅者的模板

```bash
cd ~/ros2_ws/src
ros2 pkg create publish_pkg --build-type ament_cmake --dependencies rclcpp std_msgs
cd publish_pkg
```

模板中包名为`subscription_pkg`,节点名为`subscriptionNode`,订阅的话提名为`temp_topic`

编译方法,把src复制到linux下项目目录下,执行`colcon build`或者` colcon build --packages-select subscription_pkg`

然后运行:`source install/setup.bash`然后`ros2 run subscription_pkg subscriptionNode`

检验
```bash
wub@wub:~/code/ros_humble/subscriber_test$ source install/setup.bash 
wub@wub:~/code/ros_humble/subscriber_test$ ros2 run subscription_pkg subscriptionNode
[INFO] [1714118445.546843737] [my_subscriber]: create subscriber
[INFO] [1714118447.327586681] [my_subscriber]: data:abcdefg
[INFO] [1714118450.327749098] [my_subscriber]: data:abcdefg
[INFO] [1714118453.328203401] [my_subscriber]: data:abcdefg
```



