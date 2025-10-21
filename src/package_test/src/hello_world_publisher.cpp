#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

class HelloWorldPublisher : public rclcpp::Node
{
public:
  HelloWorldPublisher() : Node("hello_world_publisher"), count_(0) // 节点名
  {
    // 创建发布者，发布到/say话题，消息类型为std_msgs::msg::String，队列大小10
    //话题名：/say
    publisher_ = this->create_publisher<std_msgs::msg::String>("/say", 10);
    // // 话题名：/hello_world_publisher/say
    // publisher_ = this->create_publisher<std_msgs::msg::String>("say", 10);
    // 创建定时器，周期500ms，绑定回调函数
    timer_ = this->create_wall_timer(500ms, std::bind(&HelloWorldPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "hello world! " + std::to_string(count_++); // 消息内容
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); // 打印日志
    publisher_->publish(message); // 发布消息
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // 初始化ROS2
  auto node = std::make_shared<HelloWorldPublisher>(); // 创建节点实例
  rclcpp::spin(node); // 保持节点运行，等待回调
  rclcpp::shutdown();
  return 0;
}
