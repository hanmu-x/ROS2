#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class ClientNode : public rclcpp::Node
{
public:
  ClientNode(const rclcpp::NodeOptions &options) : Node("client_node", options)
  {
    client_ = this->create_client<AddTwoInts>("add_two_ints");
    // 循环等待服务的响应
    // wait_for_service:在指定的时间内服务变为可用，则函数返回 true
    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
    }

    // 构造请求
    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = 10;
    request->b = 20;

    // 发送请求
    // async_send_request 异步发送请求，并将返回的 Future 存储在 future_result 中
    auto future_result = client_->async_send_request(request);
    // 等待响应
    // rclcpp::spin_until_future_complete 函数等待异步操作完成
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = future_result.get();
      RCLCPP_INFO(this->get_logger(), "Response received: Sum = %ld", (long int)result->sum);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive response.");
    }
  }

private:
  rclcpp::Client<AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClientNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
