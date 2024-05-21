
这是一个ros2发布者的模板

ros2 pkg create publish_pkg --build-type ament_cmake --node-name publishNode --dependencies std_msgs rclcpp


模板中包名为`publish_pkg`,节点名为`publishNode`,发布的话提名为`temp_topic`

编译方法,把src复制到linux下项目目录下,执行`colcon build`或者` colcon build --packages-select publish_pkg`

然后运行:`source install/setup.bash`然后`ros2 run publish_pkg publishNode`

检验
```bash
wub@wub:~$ ros2 topic list
/parameter_events
/rosout
/temp_topic
wub@wub:~$ ros2 topic  echo temp_topic
data: abcdefg
---
data: abcdefg
---
```

