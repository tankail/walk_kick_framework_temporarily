#include <rclcpp/rclcpp.hpp>
#include "hightorque_oled/oled_interface.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("oled_run");

    oled_mission(node);

    rclcpp::shutdown();
    return 0;
}
