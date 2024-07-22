/*
 * @Author: yingjie_wang 2778809626@qq.com
 * @Date: 2024-06-16 11:59:27
 * @LastEditors: yingjie_wang 2778809626@qq.com
 * @LastEditTime: 2024-07-22 14:51:52
 * @FilePath: \tf2_listener_template\src\listener_pkg\src\listenerNode.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class StaticTransformListener : public rclcpp::Node
{
public:
  StaticTransformListener() : Node("static_transform_listener")
  {
    // 订阅静态坐标变换消息
    subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", 10, 
        std::bind(&StaticTransformListener::tfCallback, this, std::placeholders::_1));
  }

private:
  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    // 打印接收到的静态坐标变换消息
    for (auto transform : msg->transforms)
    {
      RCLCPP_INFO(get_logger(), "Received transform:");
      RCLCPP_INFO(get_logger(), "  Parent Frame ID: %s", transform.header.frame_id.c_str());
      RCLCPP_INFO(get_logger(), "  Child Frame ID: %s", transform.child_frame_id.c_str());
      RCLCPP_INFO(get_logger(), "  Translation: [%.2f, %.2f, %.2f]",
                  transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z);
      RCLCPP_INFO(get_logger(), "  Rotation: [%.2f, %.2f, %.2f, %.2f]",
                  transform.transform.rotation.x,
                  transform.transform.rotation.y,
                  transform.transform.rotation.z,
                  transform.transform.rotation.w);
    }
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StaticTransformListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
