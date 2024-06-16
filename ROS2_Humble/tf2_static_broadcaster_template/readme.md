

ros2 pkg create broadcaster_static_pkg --build-type ament_cmake --node-name broadcasterStaticNode --dependencies geometry_msgs rclcpp tf2 tf2_ros



colcon build --packages-select broadcaster_static_pkg broadcasterStaticNode


source install/setup.bash 
ros2 run broadcaster_static_pkg broadcasterStaticNode turtle1 1.0 2.0 0.0 0.0 0.0 1.57


wub@wub-Y7000P:~$ ros2 topic list
/parameter_events
/rosout
/tf_static
wub@wub-Y7000P:~$ ros2 topic echo /tf_static
transforms:
- header:
    stamp:
      sec: 1718509634
      nanosec: 835745237
    frame_id: world
  child_frame_id: turtle1
  transform:
    translation:
      x: 1.0
      y: 2.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.706825181105366
      w: 0.7073882691671998
---










