#include <rclcpp/rclcpp.hpp>

class MyDynamicNode : public rclcpp::Node {
public:
    MyDynamicNode() : Node("my_dynamic_node") {
        // 声明参数并指定默认值
        this->declare_parameter("my_int_param", 42);
        this->declare_parameter("my_double_param", 3.14);
        this->declare_parameter("my_string_param", "hello");         
    }
    
    void PrintParameters()
    {
        get_parameter("my_int_param", my_int_param_);
        get_parameter("my_double_param", my_double_param_);
        get_parameter("my_string_param", my_string_param_);
        RCLCPP_INFO(this->get_logger(), "my_int_param: %d", my_int_param_);
        RCLCPP_INFO(this->get_logger(), "my_double_param: %f", my_double_param_);
        RCLCPP_INFO(this->get_logger(), "my_string_param: %s", my_string_param_.c_str());
    }
private:
    int my_int_param_;
    double my_double_param_;
    std::string my_string_param_;
};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyDynamicNode>();
    auto synchronous_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    synchronous_client->load_parameters("/home/yahboom/ros2_ws/src/ros_two_dynamic_parameter/config/my_params.yaml");
    node->set_parameter(rclcpp::Parameter("my_int_param", 50));
    node->PrintParameters();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
