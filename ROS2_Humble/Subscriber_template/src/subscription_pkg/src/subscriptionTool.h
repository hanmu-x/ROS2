#include "rclcpp/rclcpp.hpp"
// 1. 导入消息类型
#include "std_msgs/msg/string.hpp"

// 创建一个订阅者的类
class subscriber : public rclcpp::Node
{
private:
    // 声明一个话题的订阅者,发布者为sub_er
    // clcpp::Subscription是一个模板类，要订阅的话题类型为std_msgs::msg::String
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_er;
    // 声明一个订阅者的回调函数，回调函数用于当我们接收到发布者的数据后做的反应
    // 参数是一个共享指针，指针的类型就是发布者发布的类型
    void callback(const std_msgs::msg::String::SharedPtr publish_data)
    {
        // 收到数据后的处理逻辑
        //  这里只是把订阅到的数据打印输出一下
        RCLCPP_INFO(this->get_logger(), "data:%s", publish_data->data.c_str());
    }

public:
    subscriber(std::string name);
    ~subscriber();
};

