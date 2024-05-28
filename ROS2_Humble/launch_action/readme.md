
# launch启动文件启动节点

这里的代码是 publish_template 发布者模板的代码,使用launch启动文件启动这个节点

在launch文件夹下编写launch启动文件:publish_launch.py

在CMakeLists.txt中添加

```bash
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)
```


正常在最外层src目录 colcon build 然后

source install/setup.bash

ros2 launch publish_pkg publish_launch.py