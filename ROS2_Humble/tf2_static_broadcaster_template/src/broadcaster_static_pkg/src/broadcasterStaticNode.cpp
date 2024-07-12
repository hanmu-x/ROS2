/*
 * @Author: yingjie_wang 2778809626@qq.com
 * @Date: 2024-06-16 11:41:48
 * @LastEditors: yingjie_wang 2778809626@qq.com
 * @LastEditTime: 2024-07-12 15:11:35
 * @FilePath: \tf2_static_broadcaster_template\src\broadcaster_static_pkg\src\broadcasterStaticNode.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <chrono> // 操作时间

class StaticFramePublisher : public rclcpp::Node
{
public:
  StaticFramePublisher(char *transformation[]) : Node("static_turtle_tf2_broadcaster")
  {
    // tf2_ros::StaticTransformBroadcaster 实例来发布静态坐标变换
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    while(true)
    {
      // 启动时发布一次静态转换
      this->make_transforms(transformation);
      std::this_thread::sleep_for(std::chrono::seconds(2));  // 秒
    }
  }

private:
  void make_transforms(char *transformation[])
  {
    // 创建一个 geometry_msgs::msg::TransformStamped 类型的消息变量 t 存储发布的坐标变换信息
    geometry_msgs::msg::TransformStamped t;

    // 获取当前时间，并将其分配给消息头的时间戳
    t.header.stamp = this->get_clock()->now();

    // 设置父坐标系的名称为 "world"
    t.header.frame_id = "world";

    // 子坐标系名称取自命令行参数,名称为传入的第二个参数（即命令行参数中的 child_frame_name）
    t.child_frame_id = transformation[1];

    // 设置坐标变换的平移部分
    t.transform.translation.x = atof(transformation[2]);
    t.transform.translation.y = atof(transformation[3]);
    t.transform.translation.z = atof(transformation[4]);

    // 使用 tf2::Quaternion 来处理旋转角度并转换为四元数形式填入消息
    // 创建一个 tf2::Quaternion 对象 q，并使用传入的 roll、pitch、yaw 角度来设置四元数
    tf2::Quaternion q;
    q.setRPY(
        atof(transformation[5]),
        atof(transformation[6]),
        atof(transformation[7])
    );

    // 将四元数的各个分量分配给消息中的旋转部分
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // 使用 tf_static_broadcaster_ 发布静态坐标变换消息 t
    tf_static_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char *argv[])
{
  auto logger = rclcpp::get_logger("logger");

  // Obtain parameters from command line arguments
  if (argc != 8)
  {
    RCLCPP_INFO(
        logger, "Invalid number of parameters\nusage: "
                "$ ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster "
                "child_frame_name x y z roll pitch yaw");
    return 1;
  }

  // 检查父坐标系是否为 `world`
  if (strcmp(argv[1], "world") == 0)
  {
    RCLCPP_INFO(logger, "Your static turtle name cannot be 'world'");
    return 1;
  }

  // 初始化节点并传递参数
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
  rclcpp::shutdown();
  return 0;
}