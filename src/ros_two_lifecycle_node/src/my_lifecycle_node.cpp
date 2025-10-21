#include "ros_two_lifecycle_node/my_lifecycle_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <lifecycle_msgs/msg/state.hpp> 
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

namespace my_lifecycle_pkg
{

MyLifecycleNode::MyLifecycleNode(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("my_lifecycle_node", "", options),
  count_(0)
{
  // 声明参数
  this->declare_parameter("publish_topic", "/motion_state");
  this->declare_parameter("subscribe_topic", "/cmd_vel");
  

  
  RCLCPP_INFO(this->get_logger(), "MyLifecycleNode constructed");
}

MyLifecycleNode::~MyLifecycleNode()
{
  RCLCPP_INFO(this->get_logger(), "MyLifecycleNode destroyed");
}

nav2_util::CallbackReturn MyLifecycleNode::on_configure(const rclcpp_lifecycle::State & state)
{
      // 获取参数值
  publish_topic_ = this->get_parameter("publish_topic").as_string();
  subscribe_topic_ = this->get_parameter("subscribe_topic").as_string();
  (void)state;
  RCLCPP_INFO(this->get_logger(), "on_configure() called");
  
  // 创建发布者（生命周期发布者）
  publisher_ = this->create_publisher<std_msgs::msg::String>(publish_topic_, 10);
  
  // 创建订阅者
  subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    subscribe_topic_, 10,
    std::bind(&MyLifecycleNode::subscription_callback, this, std::placeholders::_1));
  
  // 创建定时器（但不立即启动）
  timer_ = this->create_wall_timer(
    1000ms, std::bind(&MyLifecycleNode::timer_callback, this));
  
  // 初始状态下停止定时器
  timer_->cancel();
  
  RCLCPP_INFO(this->get_logger(), "Configuration completed successfully");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MyLifecycleNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "on_activate() called");
  
  // 激活发布者
  publisher_->on_activate();
  
  // 启动定时器
  timer_->reset();
  
  RCLCPP_INFO(this->get_logger(), "Node activated");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MyLifecycleNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "on_deactivate() called");
  
  // 停止定时器
  timer_->cancel();
  
  // 停用发布者
  publisher_->on_deactivate();
  
  RCLCPP_INFO(this->get_logger(), "Node deactivated");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MyLifecycleNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "on_cleanup() called");
  
  // 清理资源
  timer_.reset();
  publisher_.reset();
  subscription_.reset();
  
  count_ = 0;
  
  RCLCPP_INFO(this->get_logger(), "Node cleaned up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MyLifecycleNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "on_shutdown() called");
  
  // 关闭时清理资源
  timer_.reset();
  publisher_.reset();
  subscription_.reset();
  
  RCLCPP_INFO(this->get_logger(), "Node shutdown completed");
  return nav2_util::CallbackReturn::SUCCESS;
}

void MyLifecycleNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS2! Count: " + std::to_string(count_++);
  
  // 只有在激活状态下才发布消息
  if (publisher_->is_activated()) {
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  }
}

void MyLifecycleNode::subscription_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), 
    "Received Twist: linear.x=%.2f, angular.z=%.2f", 
    msg->linear.x, msg->angular.z
  );
}

}  // namespace my_lifecycle_pkg

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<my_lifecycle_pkg::MyLifecycleNode>();
    
    // 加载参数
    auto synchronous_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    synchronous_client->load_parameters("/home/yahboom/ros2_ws/src/ros_two_lifecycle_node/config/my_params.yaml");

    nav2_util::CallbackReturn ret_;

    // 获取当前状态
    auto current_state = node->get_current_state();
    
    // 如果当前状态是 UNCONFIGURED，则配置节点
    if (current_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
        node->configure(ret_);
        if (ret_ != nav2_util::CallbackReturn::SUCCESS) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("main"), "Configure failed");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Configured successfully.");
        
        // 更新状态
        current_state = node->get_current_state();
    }

    // 如果当前状态是 INACTIVE，则激活节点
    if (current_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        node->activate(ret_);
        if (ret_ != nav2_util::CallbackReturn::SUCCESS) {
            RCLCPP_ERROR(rclcpp::get_logger("main"), "Activate failed");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Activated successfully.");
        
        // 更新状态
        current_state = node->get_current_state();
    }

    // 等待一段时间
    RCLCPP_INFO(node->get_logger(), "Node running for 10 seconds...");
    std::this_thread::sleep_for(std::chrono::seconds(10));
    
    // 正确的状态转换顺序：active -> inactive -> unconfigured -> finalized
    if (current_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        // 先停用
        node->deactivate(ret_);
        if (ret_ != nav2_util::CallbackReturn::SUCCESS) {
            RCLCPP_ERROR(rclcpp::get_logger("main"), "Deactivate failed");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Deactivated successfully.");
        
        // 更新状态
        current_state = node->get_current_state();
    }
    
    if (current_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        // 再清理
        node->cleanup(ret_);
        if (ret_ != nav2_util::CallbackReturn::SUCCESS) {
            RCLCPP_ERROR(rclcpp::get_logger("main"), "Cleanup failed");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Cleaned up successfully.");
        
        // 更新状态
        current_state = node->get_current_state();
    }
    
    if (current_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
        // 最后关闭
        node->shutdown(ret_);
        if (ret_ != nav2_util::CallbackReturn::SUCCESS) {
            RCLCPP_ERROR(rclcpp::get_logger("main"), "Shutdown failed");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Shutdown completed successfully.");
    }

    rclcpp::shutdown();
    return 0;
}
