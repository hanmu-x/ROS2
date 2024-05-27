#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam() : Node("minimal_param_node")
  {
    // 该节点在构造函数中声明了一个参数"my_parameter"，默认值为"world"
    // 这个参数会存储在ROS2的参数服务器中
    this->declare_parameter("my_parameter", 1);
    // 它创建了一个定时器，每隔1000毫秒（1秒）触发一次回调函数timer_callback()
    timer_ = this->create_wall_timer(
        1000ms,
        std::bind(&MinimalParam::timer_callback, this));
  }

  void timer_callback()
  {
    // 获取名为"myparameter"的参数的值
    int my_param = this->get_parameter("my_parameter").as_int();

    RCLCPP_INFO(this->get_logger(), "Hello %d!", my_param);

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", ++my_param)};
    // 更新参数的值到参数服务器上
    this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}
