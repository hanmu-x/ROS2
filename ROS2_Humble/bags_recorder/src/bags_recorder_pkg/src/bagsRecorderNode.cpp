
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rosbag2_cpp/writer.hpp>
// 引入自定义数据类型
#include "publish_msg/msg/my_custom_msg.hpp"

// 记录多个话题
class MultiTopicBagRecorder : public rclcpp::Node
{
public:
  MultiTopicBagRecorder() : Node("multi_topic_bag_recorder")
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("my_bag");

    // 为每个话题创建订阅者
    subscription1_ = create_subscription<std_msgs::msg::String>(
        "temp_topic", 10, std::bind(&MultiTopicBagRecorder::topic1_callback, this, std::placeholders::_1));
    
    subscription2_ = create_subscription<publish_msg::msg::MyCustomMsg>(
        "my_topic", 10, std::bind(&MultiTopicBagRecorder::topic2_callback, this, std::placeholders::_1));
    // subscription3_ = create_subscription<std_msgs::msg::String>(
    //     "temp_topic3", 10, std::bind(&MultiTopicBagRecorder::topic3_callback, this, std::placeholders::_1));
  }

private:

  void topic1_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    rclcpp::Time time_stamp = this->now();
    writer_->write(msg, "temp_topic", "std_msgs/msg/String", time_stamp);
  }

  void topic2_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    rclcpp::Time time_stamp = this->now();
    writer_->write(msg, "my_topic", "publish_msg/msg/MyCustomMsg", time_stamp);
  }

  // void topic3_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  // {
  //   rclcpp::Time time_stamp = this->now();
  //   writer_->write(msg, "temp_topic3", "std_msgs/msg/String", time_stamp);
  // }

  // 创建针对不同话题的订阅者
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription1_;
  // 订阅自定义数据类型
  rclcpp::Subscription<publish_msg::msg::MyCustomMsg>::SharedPtr subscription2_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription3_;
  
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiTopicBagRecorder>());
  rclcpp::shutdown();
  return 0;
}


// 记录一个话题
// class SimpleBagRecorder : public rclcpp::Node
// {
// public:
//   SimpleBagRecorder() : Node("simple_bag_recorder")
//   {
//     writer_ = std::make_unique<rosbag2_cpp::Writer>();
//     writer_->open("my_bag");
//     // 订阅主题"temp_topic"，该主题的消息类型为std_msgs::msg::String，队列大小为10，回调函数为topic_callback
//     subscription_ = create_subscription<std_msgs::msg::String>(
//         "temp_topic", 10, std::bind(&SimpleBagRecorder::topic_callback, this,  std::placeholders::_1));
//   }
// private:
//   void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
//   {
//     rclcpp::Time time_stamp = this->now();
//     writer_->write(msg, "temp_topic", "std_msgs/msg/String", time_stamp);
//   }
//   // 创建订阅者
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
//   // 创建 rosbag2_cpp::Writer的唯一指针实例，用于写入ROS Bag数据
//   std::unique_ptr<rosbag2_cpp::Writer> writer_;
// };
// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<SimpleBagRecorder>());
//   rclcpp::shutdown();
//   return 0;
// }



/////////////////



