
#include "publishTool.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  // 声明一个节点对象，该节点的类型是我们是声明的 发布者类
  auto node = std::make_shared<publisher>("my_publishr");
  while(1)
  {
    // 循环发布数据
    sleep(3);
    node->publish_data();
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
}
