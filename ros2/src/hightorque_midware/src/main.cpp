#include <rclcpp/rclcpp.hpp>
#include "motor_manager_node.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<hightorque::MotorManagerNode>();
        node->init();

        RCLCPP_INFO(node->get_logger(), "Motor Manager Node started");

        rclcpp::spin(node);

        RCLCPP_INFO(node->get_logger(), "Motor Manager Node shutting down");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_manager"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
