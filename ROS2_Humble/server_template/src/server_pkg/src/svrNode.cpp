
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class SvNode : public rclcpp::Node
{
public:
    SvNode(const rclcpp::NodeOptions &options) : Node("svrNode", options)
    {
        // 创建名为 "svrNode" 的节点，并且创建一个服务并将其绑定到名为 "add_two_ints" 的话题上
        // 服务的回调函数为 handle_request
        service_ = this->create_service<AddTwoInts>("add_two_ints", 
            std::bind(&SvNode::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // 回调函数:两个参数：请求（request）和响应（response）
    void handle_request(const std::shared_ptr<AddTwoInts::Request> request,
                        const std::shared_ptr<AddTwoInts::Response> response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request: a=%ld, b=%ld. Sum: %ld", 
                    request->a, request->b, (long int)response->sum);
    }

    rclcpp::Service<AddTwoInts>::SharedPtr service_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 创建 SvNode 的实例
    auto node = std::make_shared<SvNode>(rclcpp::NodeOptions());
    // 进入 ROS2 的事件循环
    rclcpp::spin(node);
    // 在程序退出之前，调用 rclcpp::shutdown() 关闭 ROS2 节点
    rclcpp::shutdown();
    return 0;
}