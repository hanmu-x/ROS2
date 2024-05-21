// #include "subscriptionTool.h"


#include "rclcpp/rclcpp.hpp"
#include "publish_msg/msg/my_custom_msg.hpp" // 导入您定义的消息类型

class subscriber : public rclcpp::Node
{
public:
    // 允许传入节点名，默认值提供灵活性
    subscriber(const std::string &node_name = "subscriber_node"): Node(node_name)
    {
        // 订阅名为 "my_topic" 的主题，并定义消息到达时的回调函数
        subscription_ = this->create_subscription<publish_msg::msg::MyCustomMsg>(
            "my_topic", 10, std::bind(&subscriber::message_callback, this, std::placeholders::_1));
    }

private:
    // 定义消息到达时的回调函数
    void message_callback(const publish_msg::msg::MyCustomMsg::SharedPtr msg)
    {
        // 在回调函数中处理接收到的消息
        RCLCPP_INFO(this->get_logger(), "Received: %d, '%s'", msg->my_integer, msg->my_string.c_str());
    }

    rclcpp::Subscription<publish_msg::msg::MyCustomMsg>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化ROS 2节点
    auto node = std::make_shared<subscriber>(); // 创建订阅者节点
    rclcpp::spin(node); // 进入ROS 2事件循环
    rclcpp::shutdown(); // 关闭ROS 2节点
    return 0;
}





