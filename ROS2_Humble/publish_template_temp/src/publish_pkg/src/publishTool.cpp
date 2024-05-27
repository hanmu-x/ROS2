#include "publishTool.h"

publisher::publisher(std::string name) : Node(name)
{
  // 发布者的构造函数
  RCLCPP_INFO(this->get_logger(), "create publisher");

  // 3. 创造一个发布者
  // 使用create_publisher这个函数创建发布者对象，并声明一下他的发布的数据类型
  // 两个参数，
    // 第一个是发布的话题名称，
    // 第二个是队列长度,当发布者发布消息的速度快于订阅者接收消息的速度时，消息会被暂时存储在发布队列中。这里的 10 表示发布队列的长度为 10，即最多可以存储 10 条待发布的消息。如果队列已满而发布者尝试发布新的消息，那么最早进入队列的消息将被丢弃
  pub_er = this->create_publisher<std_msgs::msg::String>("temp_topic", 10);
}

// 析构函数，该函数在释放对象后调用，或者按CTRL+C后调用
publisher::~publisher()
{
  RCLCPP_INFO(this->get_logger(), "delete publisher");
}

// 调用该函数，使用发布者把数据发布出去
void publisher::publish_data()
{
  // 声明一个存储数据的对象
  std_msgs::msg::String text_data;
  // 向对象中存储数据要调用该数据类型对象的data
  text_data.data = "abcdefg";
  pub_er->publish(text_data);
}