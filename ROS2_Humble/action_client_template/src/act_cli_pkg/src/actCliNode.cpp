#include <functional>
#include <future>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_pkg/action/rotate.hpp"

using Rotate = action_pkg::action::Rotate;
using GoalHandleRotate = rclcpp_action::ClientGoalHandle<Rotate>;

class RotateActionClient : public rclcpp::Node
{
public:
  explicit RotateActionClient(const rclcpp::NodeOptions &options) : Node("rotate_action_client", options)
  {
    // 函数创建了一个动作客户端，指定了节点实例指针 this 和动作服务器的名称 "rotate"
    client_ptr_ = rclcpp_action::create_client<Rotate>(this, "rotate");
    // 循环等待动作服务器上线，直到 wait_for_action_server 返回 true，表示服务器已经上线。
    while (!client_ptr_->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(get_logger(), "Waiting for action server to be up...");
    }
  }

  // 用于向动作服务器发送旋转目标
  void send_goal(int degrees)
  {
    auto goal_msg = Rotate::Goal();
    goal_msg.goal = degrees;

    // using namespace std::placeholders;
    // goal_response_callback 用于处理目标响应
    auto send_goal_options = rclcpp_action::Client<Rotate>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&RotateActionClient::goal_response_callback, this, std::placeholders::_1);
    // feedback_callback 用于处理反馈消息
    send_goal_options.feedback_callback = std::bind(&RotateActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    // result_callback 用于处理目标完成后的结果
    send_goal_options.result_callback = std::bind(&RotateActionClient::result_callback, this, std::placeholders::_1);
    // 调用客户端的 async_send_goal (异步)函数发送目标，并将返回的 std::shared_future 对象存储在 goal_handle_future_ 成员中
    goal_handle_future_ = client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  // wait_for_result 函数用于等待目标完成。它调用 goal_handle_future_ 对象的 wait 函数，阻塞当前线程直到目标完成
  void wait_for_result()
  {
    goal_handle_future_.wait();
  }

private:
  rclcpp_action::Client<Rotate>::SharedPtr client_ptr_;                                       // 处理action的客户端 的智能指针
  std::shared_future<rclcpp_action::ClientGoalHandle<Rotate>::SharedPtr> goal_handle_future_; // 存储未来获取的action目标句柄

  // 当action目标被服务器接受或拒绝时被调用
  void goal_response_callback(rclcpp_action::ClientGoalHandle<Rotate>::SharedPtr goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    }
  }
  // 反馈回调函数，用于接收action执行过程中的反馈信息
  void feedback_callback(GoalHandleRotate::SharedPtr, const std::shared_ptr<const Rotate::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Current rotation angle: %d degrees", feedback->feedback);
  }
  // 结果回调函数，用于处理action执行结束后的结果
  void result_callback(const rclcpp_action::ClientGoalHandle<Rotate>::WrappedResult &result)
  {
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      printf("client Goal succeeded with final angle: %d degrees \n", result.result->result);
      RCLCPP_INFO(get_logger(), "client Goal succeeded with final angle: %d degrees", result.result->result);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      return;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions().allow_undeclared_parameters(true);
  auto client = std::make_shared<RotateActionClient>(options);

  // 请求旋转270度
  client->send_goal(270);

  // 使用新的公共方法等待结果
  client->wait_for_result();

  rclcpp::shutdown();
  return 0;
}