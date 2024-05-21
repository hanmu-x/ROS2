
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class SvNode : public rclcpp::Node
{
public:
    SvNode(const rclcpp::NodeOptions &options) : Node("svrNode", options)
    {
        service_ = this->create_service<AddTwoInts>("add_two_ints", 
            std::bind(&SvNode::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
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
    auto node = std::make_shared<SvNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}