#include "no_op_kinematics_plugin.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace hightorque
{

    bool NoOpKinematicsPlugin::configure(const KinematicsPluginConfig& config)
    {
        config_ = config;
        return true;
    }

    bool NoOpKinematicsPlugin::forward(
        const sensor_msgs::msg::JointState& currentState,
        sensor_msgs::msg::JointState& publishedState)
    {
        publishedState = currentState;
        return true;
    }

    bool NoOpKinematicsPlugin::inverse(
        const sensor_msgs::msg::JointState& currentState,
        const hightorque_msgs::msg::MotorControlCommand& command,
        hightorque_msgs::msg::MotorControlCommand& inversedCommand)
    {
        (void)currentState;

        inversedCommand = command;
        if (inversedCommand.motor_ids.empty())
        {
            inversedCommand.motor_ids = config_.motorIds;
        }

        return true;
    }

    const KinematicsPluginConfig& NoOpKinematicsPlugin::config() const
    {
        return config_;
    }

} // namespace hightorque

PLUGINLIB_EXPORT_CLASS(hightorque::NoOpKinematicsPlugin, hightorque::KinematicsPlugin)
