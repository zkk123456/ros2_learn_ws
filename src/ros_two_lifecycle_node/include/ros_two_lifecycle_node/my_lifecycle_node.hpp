#ifndef MY_LIFECYCLE_PKG__MY_LIFECYCLE_NODE_HPP_
#define MY_LIFECYCLE_PKG__MY_LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace my_lifecycle_pkg
{
class MyLifecycleNode : public nav2_util::LifecycleNode
{
public:
  explicit MyLifecycleNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MyLifecycleNode();
  // 生命周期管理
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
private:
  // 定时器回调函数
  void timer_callback();  
  // 订阅者回调函数
  void subscription_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  // ROS2相关成员变量  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;  
  // 计数器
  size_t count_;  
  // 话题名称
  std::string publish_topic_;
  std::string subscribe_topic_;  
};
}  // namespace my_lifecycle_pkg

#endif  // MY_LIFECYCLE_PKG__MY_LIFECYCLE_NODE_HPP_
