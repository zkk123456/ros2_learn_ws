#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <chrono>
#include "ros_two_interface/action/count_numbers.hpp"  // 替换为你的包名

using namespace std::chrono_literals;
using CountNumbers = ros_two_interface::action::CountNumbers;  // 替换为你的包名
using GoalHandleCountNumbers = rclcpp_action::ServerGoalHandle<CountNumbers>;

class CountActionServer : public rclcpp::Node
{
public:
    CountActionServer() : Node("count_action_server")
    {
        // 创建Action Server
        action_server_ = rclcpp_action::create_server<CountNumbers>(
            this,
            "count_numbers",
            std::bind(&CountActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CountActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&CountActionServer::handle_accepted, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Count Action Server 已启动");
    }

private:
    rclcpp_action::Server<CountNumbers>::SharedPtr action_server_;

    // 处理新目标请求
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const CountNumbers::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "收到新目标: %.1f", goal->target_num);
        (void)uuid;
        
        // 检查目标是否有效
        if (goal->target_num <= 0) {
            RCLCPP_ERROR(this->get_logger(), "目标数字必须大于0");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 处理取消请求
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleCountNumbers> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "收到取消请求");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 接受目标并开始执行
    void handle_accepted(const std::shared_ptr<GoalHandleCountNumbers> goal_handle)
    {
        // 在新线程中执行计数任务
        std::thread{std::bind(&CountActionServer::execute_count, this, std::placeholders::_1), goal_handle}.detach();
    }

    // 执行计数任务
    void execute_count(const std::shared_ptr<GoalHandleCountNumbers> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "开始执行计数任务");
        
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<CountNumbers::Result>();
        auto feedback = std::make_shared<CountNumbers::Feedback>();
        
        float current = 0.0;
        
        try {
            while (rclcpp::ok() && current < goal->target_num) {
                // 检查是否被取消
                if (goal_handle->is_canceling()) {
                    result->result = "cancel";
                    result->current_num = current;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "计数任务被取消");
                    return;
                }
                
                // 更新当前数字
                current += 1.0;
                feedback->current_num = current;
                
                // 发布反馈
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "当前计数: %.1f", current);
                
                // 等待1秒
                std::this_thread::sleep_for(1000ms);
            }
            
            // 任务完成
            if (rclcpp::ok()) {
                result->result = "success";
                result->current_num = current;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "计数任务完成，最终数字: %.1f", current);
            }
            
        } catch (const std::exception& e) {
            result->result = "failed";
            result->current_num = current;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "计数任务失败: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
