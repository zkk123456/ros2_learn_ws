#include "rclcpp/rclcpp.hpp"
#include "ros_two_interface/srv/add_two_ints.hpp"
#include <memory>
using std::placeholders::_1;
using std::placeholders::_2;
class AddTwoIntsServer : public rclcpp::Node
{
public:
  AddTwoIntsServer() : Node("add_two_ints_server")
  {
    service_ = this->create_service<ros_two_interface::srv::AddTwoInts>(
      "add_two_ints",
      std::bind(&AddTwoIntsServer::handle_service, this, _1, _2));    
    RCLCPP_INFO(this->get_logger(), "AddTwoInts服务端已启动");
  }
private:
  void handle_service(
    const std::shared_ptr<ros_two_interface::srv::AddTwoInts::Request> request,
    const std::shared_ptr<ros_two_interface::srv::AddTwoInts::Response> response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "收到请求: %ld + %ld = %ld", 
                request->a, request->b, response->sum);
  }  
  rclcpp::Service<ros_two_interface::srv::AddTwoInts>::SharedPtr service_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
