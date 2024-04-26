```bash
```

# 1. 基础指令

1. `ros2 run`会从一个软件包中启动一个可执行文件。

```bash
ros2 run <package_name> <executable_name>  ## 包名  节点名
```

2. `ros2 node list` 命令将显示所有正在运行的节点的名称。
3. `ros2 node info <node_name>` 访问有关它们的更多信息 
```bash
ros2 node info /my_turtle  #节点``my_turtle``
```
4. 使用``rqt_graph``来可视化节点和话题的变化，以及它们之间的连接。
5. ``ros2 topic list``命令将返回系统中当前活动的所有话题
6. `ros2 topic list -t` 将返回相同的话题列表，这次在括号中附加了话题类型：
7. `ros2 topic echo <topic_name>` 查看在话题上发布的数据
8. `ros2 topic info <topic_name>` 查看话题的具体信息,话题不仅可以是一对一的通信，还可以是一对多的、多对一的或多对多的通信。
9. 


# 2 包和节点的创建


## 创建工作区目录

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
我们创建了两个目录，ros2_ws和在他下面的子目录，运行编译的目录在ros2_ws目录下，src目录下是我们生成的ros的包，我们自己编写的代码也都早src目录下，colcon build编译之后会在ros2_ws目录下生成一下文件，运行程序也是在ros2_ws这个目录下。

## 创建包和节点

使用`ros2 pkg create`来创建包和节点，**(该命令是在src目录下执行)**。

1. 只创建包

```bash
ros2 pkg create <package_name> --build-type ament_cmake
```

2.创建包并添加依赖

```bash
ros2 pkg create my_package --build-type ament_cmake --dependencies std_msgs rclcpp
```
- ament_cmake,适用于c++,是cmake的增强版
- ament_python，适用于Python
- 创建包的同时添加`std_msgs`和`rclcpp`的依赖
 
3. 创建包的同时添加节点

```bash
ros2 pkg create <package_name> --build-type ament_cmake --node-name <node_name> --dependencies std_msgs rclcpp
```
- 创建包名填入package_name
- 创建节点名填入node_name
- 这里添加了`std_msgs`和`rclcpp`两个依赖，需要的依赖填在在后面

运行节点的命令

```bash
ros2 run 包名 可执行文件名
ros2 run package_name exectable_name
```

4. 列出所有的包

```bash
ros2 pkg list
```

5. 输出某个包的路径前缀

```bash
ros2 pkg prefix <package_name>
```

6. 列出某个包的清单信息

```bash
ros2 pkg xml <package_name>
```

## colcon build编译

我们在有src文件夹的目录下，也就是执行文件的目录

编译所有的包

```bash
colcon build
```

指定单独编译某个包：
```bash
colcon build --packages-select <package_name>
```


## CMakeLists.txt文件

该文件是告诉编译器，想要找某个节点去哪里找。

```cpp
cmake_minimum_required(VERSION 3.5)
project(ros_learn)  //包的名称
```

## find_package
如果是添加了某些依赖，需要再find_package中添加

```cpp
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msga REQUIRED)
find_package(rclcpp REQUIRED)
```

## add_executable

添加节点，表示如果编译器要找那个节点到那个目录下去找

```cpp
// 表示告诉编辑器想要找infor_publish这个节点去src目录下的infor_publish.cpp去找
add_executable(infor_publish src/infor_publish.cpp)
```

然后加上ament_target_dependencies这项，该项里记录者余姚的依赖和运行节点

```cpp
ament_target_dependencies(
  infor_publish
  "std_msga"
  "rclcpp"
)
//rclcpp和std_msga也可以不加双引号
```

然后还需要安装一下，安装一下我们的可执行节点

```cpp
install(TARGETS 
		infor_publish
  DESTINATION lib/${PROJECT_NAME})
```

## package.xml


该文件中需要添加一下我们手动添加的依赖

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ros_learn</name>   <!-- 包的名称 -->
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="aywq@todo.todo">aywq</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>rclcpp</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

```

## 面相过程的方式生命一个节点

- 要继承rclcpp::Node的节点（`auto node = std::make_shared<rclcpp::Node>("ros_1")`），然后打印一条消息

```cpp
    // 1. 导入库文件
# include "rclcpp/rclcpp.hpp"

int main(int argc,char **argv)
{
    //2.初始化客户端库
    rclcpp::init(argc,argv);
    //3.使用智能指针创建新的节点对象
    auto node = std::make_shared<rclcpp::Node>("ros_1");// 当我们初始化节点的时候要给他一个参数，给节点起一个名字

    // ros的打印
    RCLCPP_INFO(node->get_logger(),"打印内容");

    //4.使用spin循环节点
    rclcpp::spin(node);
    //5. 关闭客户端库
    rclcpp::shutdown();
}
```


## 以面向对象的方式创建一个节点

- 在面相对象的节点的创建过程中，我们要声明一个类，该类**要继承rclcpp::Node**（ `:public rclcpp::Node`），否则该类无法拥有拥有日志打印，创建服务话题的能力。
- 构造函数要给在初始化列表给父类的Node传节点的名字

```cpp
// 1. 导入库文件
# include "rclcpp/rclcpp.hpp"
# include <string>

// 定义一个类
// 先要让这个类继承rclcpp::Node，只有继承了这个  才能拥有日志打印，创建服务话题的能力
class infor_publish :public rclcpp::Node 
{
private:
    /* data */
public:
    infor_publish(std::string name);  //构造函数在定义是不用继承初始化列表
    ~infor_publish();
};
// 我们在声明构造函数是要调用父类的构造函数，给他传一个参数，参数为节点的名字
infor_publish::infor_publish(std::string name):Node(name)
{
    RCLCPP_INFO(this->get_logger(),"hahaha:%s",name.c_str()); // name是一个string类型的，我们需要把他c的字符串打印
}
infor_publish::~infor_publish()
{
    RCLCPP_INFO(this->get_logger(),"bye!!!");
}

int main(int argc,char **argv)
{
    //2.初始化客户端库
    rclcpp::init(argc,argv);
    //3.使用智能指针创建新的节点对象,在面相对象的这节点声明，智能指针就要输入类的类型，而不是rclcpp::Node
    auto node = std::make_shared<infor_publish>("ros2");

    //4.使用spin循环节点
    rclcpp::spin(node);
    //5. 关闭客户端库
    rclcpp::shutdown();
}
