#include <functional>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_pkg/action/rotate.hpp"

using Rotate = action_pkg::action::Rotate;
using GoalHandleRotate = rclcpp_action::ServerGoalHandle<Rotate>;

class RotateActionServer : public rclcpp::Node
{
public:
    // 初始化节点名为"rotate_action_server"
    RotateActionServer() : Node("rotate_action_server")
    {
        // using namespace std::placeholders;
        // 绑定处理目标、取消请求和接受目标的回调函数。
        action_server_ = rclcpp_action::create_server<Rotate>(
            this,
            "rotate",  // 定义动作服务器名称，客户端将通过这个名称来找到并发送目标给这个服务器
            std::bind(&RotateActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RotateActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&RotateActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Server<Rotate>::SharedPtr action_server_;

    // 当收到新的动作目标时被调用。它接受目标ID和共享的目标数据，并打印旋转角度信息
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Rotate::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received goal request: %d degrees", goal->goal);
        // 表示动作服务器接受这个目标请求
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    // 处理取消动作请求。当接收到取消请求时，打印消息并返回接受取消的响应。
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRotate>)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 在目标被接受后调用，启动一个新的线程来执行动作任务，传入目标句柄。
    void handle_accepted(const std::shared_ptr<GoalHandleRotate> goal_handle)
    {
        std::thread{std::bind(&RotateActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // 这是动作执行的核心逻辑。在新线程中运行，负责实际的旋转操作模拟
    void execute(const std::shared_ptr<GoalHandleRotate> goal_handle)
    {
        auto goal_v = goal_handle->get_goal();
        RCLCPP_INFO(get_logger(), "Target angle: %d",goal_v->goal );
        auto feedback = std::make_shared<Rotate::Feedback>();
        auto &current_angle = feedback->feedback;

        // 初始化当前角度为0
        current_angle = 0;

        // 发布初始反馈
        goal_handle->publish_feedback(feedback);

        // 控制客户端旋转，每0.01秒发布一次当前角度
        rclcpp::Rate loop_rate(100);
        while (rclcpp::ok() && current_angle < goal_v->goal)
        {
            if (goal_handle->is_canceling())
            {
                auto result_msg = std::make_shared<action_pkg::action::Rotate::Result>();
                goal_handle->canceled(result_msg);
                RCLCPP_INFO(get_logger(), "Goal canceled");
                return;
            }

            // 增加角度并更新反馈
            current_angle++;

            // // 添加日志打印当前角度变化
            // RCLCPP_INFO(get_logger(), "Current angle: %d degrees", current_angle);

            feedback->feedback = current_angle;
            // 发布反馈
            goal_handle->publish_feedback(feedback);
            // 等待0.01秒
            loop_rate.sleep();
        }

        // 检查是否完成目标
        if (current_angle >= goal_v->goal)
        {
            auto result = std::make_shared<Rotate::Result>();
            result->result = current_angle;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Server Goal succeeded, final angle: %d degrees", current_angle);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RotateActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}