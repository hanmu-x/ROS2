#include "subscriptionTool.h"
// #include <functional> // 引入 functional 头文件

subscriber::subscriber(std::string name) : Node(name)
{
    // 订阅者的构造函数
    RCLCPP_INFO(this->get_logger(), "create subscriber");

    // 构造函数里面创建订阅者
    // 使用create_subscription这个来给声明的订阅者来创建一个实体
    // create_subscriptionde
    // 第一个参数是：订阅的话题名称，
    // 第二个参数是队列长度, 10：这个参数指定了订阅队列的长度。订阅队列用于存储待处理的消息，当订阅者处理消息的速度慢于发布者发布消息的速度时，消息会被暂时存储在订阅队列中。这里的 10 表示订阅队列的长度为 10，即最多可以存储 10 条待处理的消息。如果队列已满而新的消息到达，旧的消息将被丢弃，以便为新消息腾出空间。
    // std::bind 函数用于绑定消息回调函数 subscriber::callback 到当前对象的实例上
    // std::placeholders::_1 表示回调函数将会接收一个参数，即接收到的消息
    sub_er = this->create_subscription<std_msgs::msg::String>("temp_topic", 10, std::bind(&subscriber::callback, this, std::placeholders::_1));
}

subscriber::~subscriber()
{
    RCLCPP_INFO(this->get_logger(), "delete subscriber");
}