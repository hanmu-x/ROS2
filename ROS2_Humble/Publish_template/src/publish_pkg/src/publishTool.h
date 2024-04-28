#include "rclcpp/rclcpp.hpp"
// 1. 导入消息类型
#include "std_msgs/msg/string.hpp"

// 定义一个有发布者的类，里面包含发布者对象
class publisher: public rclcpp::Node
{
private:
  // 2.声明一个发布者用于发布消息，这里只是定义，也可以在这里对其进行初始化，我这里在构造函数里面进行初始化
  // 发布者名称为pub_er,用共性指针声明对象，<std_msgs::msg::String>表示要发布的数据类型
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_er; 
public:
  // 定义一个发布话题的函数，声明类对象调用该函数发布数据。
  publisher(std::string name);
  ~publisher();
  
  void publish_data();
};