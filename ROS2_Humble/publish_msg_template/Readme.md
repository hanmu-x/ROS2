
# 自定义数据类型的发布者

该部分代码用于发布自定义类型的数据

publish_msg 这个包里是数据类型的包
publish_pkg 这个包是发布者代码

发布的数据类型为 publish_msg 包中msg目录下 MyCustomMsg.msg文件中定义的
    int32 类型的 my_integer
    string 类型的 my_string

测试案例中
    my_integer = 42;
    my_string = "Hello, World!";

# 编译指令

colcon build --packages-select publish_msg publish_pkg 

source install/setup.bash

ros2 run publish_pkg publishNode

