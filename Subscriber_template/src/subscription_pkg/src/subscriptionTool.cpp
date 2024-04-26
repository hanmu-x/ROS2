#include "subscriptionTool.h"
//#include <functional> // 引入 functional 头文件

subscriber::subscriber(std::string name) : Node(name)
{
    // 订阅者的构造函数
    RCLCPP_INFO(this->get_logger(), "create subscriber");

    // 构造函数里面创建订阅者
    // 使用create_subscription这个来给声明的订阅者来创建一个实体
    // create_subscriptionde第一个参数是：订阅的话题名称，第二个参数是队列长度
    // 成员函数不能只用当做参数进行回调，成员函数需要一个对象来调用，可以使用bind
    sub_er = this->create_subscription<std_msgs::msg::String>("temp_topic", 10, std::bind(&subscriber::callback, this, std::placeholders::_1));
}

subscriber::~subscriber()
{
    RCLCPP_INFO(this->get_logger(), "delete subscriber");
}