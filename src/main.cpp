#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros2_whill/ros2_whill_component.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto whill_node = std::make_shared<WhillController>();
    rclcpp::spin(whill_node);
    rclcpp::shutdown();
    return 0;
}