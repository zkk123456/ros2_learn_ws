#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include "ros_two_interface/action/count_numbers.hpp"  // 替换为你的包名
using namespace std::chrono_literals;
using CountNumbers = ros_two_interface::action::CountNumbers;
class CountActionClient : public rclcpp::Node
{
public:
    CountActionClient() : Node("count_action_client")
    {
        client_ = rclcpp_action::create_client<CountNumbers>(this, "count_numbers");
    }
    void send_goal(float target_num)
    {
        // 等待Action Server
        if (!client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action Server 未找到");
            return;
        }
        auto goal_msg = CountNumbers::Goal();
        goal_msg.target_num = target_num;
        RCLCPP_INFO(this->get_logger(), "发送目标: %.1f", target_num);
        // 设置选项
        auto send_goal_options = rclcpp_action::Client<CountNumbers>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            [this](GoalHandleCountNumbers::SharedPtr, const std::shared_ptr<const CountNumbers::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "收到反馈: 当前数字 = %.1f", feedback->current_num);
            };        
        send_goal_options.result_callback = 
            [this](const GoalHandleCountNumbers::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "任务成功: %s, 最终数字: %.1f", 
                                   result.result->result.c_str(), result.result->current_num);
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "任务失败: %s", result.result->result.c_str());
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "任务取消: %s", result.result->result.c_str());
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "未知结果");
                        break;
                }
                rclcpp::shutdown();
            };
        // 发送目标
        client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<CountNumbers>::SharedPtr client_;
    using GoalHandleCountNumbers = rclcpp_action::ClientGoalHandle<CountNumbers>;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto client_node = std::make_shared<CountActionClient>();
    
    // 从命令行参数获取目标数字，默认为5
    float target_num = 5.0;
    if (argc > 1) {
        target_num = std::stof(argv[1]);
    }    
    client_node->send_goal(target_num);
    rclcpp::spin(client_node);    
    return 0;
}
