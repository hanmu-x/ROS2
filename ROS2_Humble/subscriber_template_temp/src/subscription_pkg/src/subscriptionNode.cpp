#include "subscriptionTool.h"


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<subscriber>("my_subscriber");
  rclcpp::spin(node);
  rclcpp::shutdown();
}