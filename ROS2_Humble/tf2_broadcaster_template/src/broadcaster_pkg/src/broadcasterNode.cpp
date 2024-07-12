
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher() : Node("turtle_tf2_frame_publisher")
  {
    // declare_parameter 获取名为 turtlename 的节点参数，如果不存在则默认为 "turtle"
    turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle1");

    // 用于发布TF变换
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 订阅turtle{1}{2}/pose主题并调用handle_turtle_pose
    // 每个消息的回调函数
    std::ostringstream stream;
    stream << "/" << turtlename_.c_str() << "/pose";
    std::string topic_name = stream.str();

    // 创建订阅器 (subscription_) 订阅 /turtle{1}{2}/pose 主题，每当有消息到达时，调用 handle_turtle_pose 处理回调函数
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
        topic_name, 10,
        std::bind(&FramePublisher::handle_turtle_pose, this, std::placeholders::_1));
  }

private:
  void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
  {
    // 创建 geometry_msgs::msg::TransformStamped 对象 t，用于存储TF变换信息
    geometry_msgs::msg::TransformStamped t;

    // 从收到的 msg 中获取乌龟的位置 (x 和 y 坐标)，将 z 坐标设为0
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = turtlename_.c_str();

    // Turtle只存在于2D中，因此我们得到x和y的平移
    // 从消息中获取坐标，并将z坐标设置为0
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    // 同理，turtle只能绕一个轴旋转
    // 这就是为什么我们将x和y的旋转设置为0并获得
    // 在z轴上旋转
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string turtlename_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}