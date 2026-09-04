#include <rclcpp/rclcpp.hpp>
#include "ifssim_ros_wrapper.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("ifssim_bridge");

    std::string host = node->declare_parameter<std::string>("host", "127.0.0.1");
    int port = node->declare_parameter<int>("port", 41451);
    double timeout = node->declare_parameter<double>("timeout", 5.0);

    RCLCPP_INFO(node->get_logger(), "IFSSIM ROS2 Bridge starting — connecting to %s:%d", host.c_str(), port);

    IFSSIMRosWrapper wrapper(node, host, port, timeout);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
