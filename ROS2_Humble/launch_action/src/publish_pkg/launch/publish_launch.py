# 导入launch启动文件所需的库
from launch import LaunchDescription
from launch_ros.actions import Node

# 定义一个generate_launch_description()的函数
def generate_launch_description():
    publisher_node = Node(
        package="publish_pkg", # 要运行的包名
        executable="publishNode", # 要运行节点的可执行文件的名称
        name="pub_node"
    )
    # 可直接添加下一个可执行文件
    # publisher_node_2 = Node(
    #     package="publisher_ros2", # 要运行的包名
    #     executable="my_publisher" # 要运行节点的可执行文件的名称
    # )

    # 创建一个LaunchDescription的对象launch_des，用于描述launch文件
    return LaunchDescription([
        publisher_node
        # publisher_node_2
    ])













