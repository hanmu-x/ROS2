#include "rclcpp/rclcpp.hpp"
// 1. 导入消息类型
#include "std_msgs/msg/string.hpp"

#include "publish_msg/msg/my_custom_msg.hpp"

class publisher : public rclcpp::Node // 类名保持一致，建议用publisher而非publisher以避免与rclcpp::Publisher混淆
{
public:
    // 允许传入节点名，默认值提供灵活性
    publisher(const std::string &node_name = "publisher_node"): Node(node_name)
    {
        publisher_ = this->create_publisher<publish_msg::msg::MyCustomMsg>("my_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&publisher::publish_message, this));
    }

private:
    void publish_message()
    {
        auto message = publish_msg::msg::MyCustomMsg();
        message.my_integer = 42;
        message.my_string = "Hello, World!";
        RCLCPP_INFO(this->get_logger(), "Publishing: %d, '%s'", message.my_integer, message.my_string.c_str());
        publisher_->publish(message);
    }
    rclcpp::Publisher<publish_msg::msg::MyCustomMsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};








// // 定义一个有发布者的类，里面包含发布者对象
// class publisher : public rclcpp::Node
// {
// private:
//   // 2.声明一个发布者用于发布消息，这里只是定义，也可以在这里对其进行初始化，我这里在构造函数里面进行初始化
//   // 发布者名称为pub_er,用共性指针声明对象，<std_msgs::msg::String>表示要发布的数据类型
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_er;

// public:
//   // 定义一个发布话题的函数，声明类对象调用该函数发布数据。
//   publisher(std::string name);
//   ~publisher();

//   void publish_data();
// };