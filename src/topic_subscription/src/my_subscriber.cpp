#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
/**
* 创建一个继承自rclcpp::Node的类MinimalSubscriber
* 这样可以使用ROS2节点的所有功能。
*/
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber") // 节点构造函数，设置节点名称为"minimal_subscriber"
  {
    // 创建订阅者，订阅名为"/say"的话题，消息类型为std_msgs::msg::String
    // 10是队列长度，std::bind将成员函数topic_callback绑定为消息到达时的回调函数
    // _1是一个占位符，表示回调函数需要一个参数
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/say", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
  }
private:
  /**
   * 定义消息回调函数
   * 当有消息发布到"/say"话题时，这个函数会被自动调用
   * @param msg 接收到的String消息的常量共享指针
   */
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    // 在终端打印接收到的消息内容
    // RCLCPP_INFO是ROS2中用于打印信息日志的宏
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  
  // 声明订阅者成员变量
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
/**
* 程序主函数
*/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // 初始化ROS2 C++客户端库
  rclcpp::spin(std::make_shared<MinimalSubscriber>()); // 创建节点实例并进入自旋，等待消息到来
  rclcpp::shutdown(); // 关闭ROS2
  return 0;
}
