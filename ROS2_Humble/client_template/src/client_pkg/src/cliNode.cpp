#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class ClientNode : public rclcpp::Node
{
public:
    ClientNode(const rclcpp::NodeOptions &options) : Node("client_node", options)
    {
        client_ = this->create_client<AddTwoInts>("add_two_ints");
        // 等待服务可用
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
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
        auto future_result = client_->async_send_request(request);
        // 等待响应
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future_result.get();
            RCLCPP_INFO(this->get_logger(), "Response received: Sum = %ld", (long int)result->sum);
        } else {
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
