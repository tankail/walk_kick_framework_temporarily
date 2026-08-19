#include "head_invert_kinematics_plugin.hpp"

#include <algorithm>

#include "pluginlib/class_list_macros.hpp"

namespace hightorque
{

    bool HeadInvertKinematicsPlugin::configure(const KinematicsPluginConfig& config)
    {
        config_ = config;
        invertedMotorIds_.clear();
        headJointNames_ = {"head_yaw_joint", "head_pitch_joint"};

        const std::size_t pairCount = std::min(config_.jointNames.size(), config_.motorIds.size());
        for (std::size_t index = 0; index < pairCount; ++index)
        {
            if (isHeadJoint(config_.jointNames[index]))
            {
                invertedMotorIds_.push_back(config_.motorIds[index]);
            }
        }

        return true;
    }

    bool HeadInvertKinematicsPlugin::forward(
        const sensor_msgs::msg::JointState& currentState,
        sensor_msgs::msg::JointState& publishedState)
    {
        publishedState = currentState;

        for (std::size_t index = 0; index < publishedState.name.size(); ++index)
        {
            if (isHeadJoint(publishedState.name[index]))
            {
                invertJointStateValue(publishedState, index);
            }
        }

        return true;
    }

    bool HeadInvertKinematicsPlugin::inverse(
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

        for (std::size_t index = 0; index < inversedCommand.motor_ids.size(); ++index)
        {
            const int32_t motorId = inversedCommand.motor_ids[index];
            if (std::find(invertedMotorIds_.begin(), invertedMotorIds_.end(), motorId) != invertedMotorIds_.end())
            {
                invertMotorCommandValue(inversedCommand, index);
            }
        }

        return true;
    }

    const KinematicsPluginConfig& HeadInvertKinematicsPlugin::config() const
    {
        return config_;
    }

    bool HeadInvertKinematicsPlugin::isHeadJoint(const std::string& jointName) const
    {
        return headJointNames_.find(jointName) != headJointNames_.end();
    }

    void HeadInvertKinematicsPlugin::invertJointStateValue(
        sensor_msgs::msg::JointState& jointState,
        std::size_t index) const
    {
        if (index < jointState.position.size())
        {
            jointState.position[index] = -jointState.position[index];
        }
        if (index < jointState.velocity.size())
        {
            jointState.velocity[index] = -jointState.velocity[index];
        }
        if (index < jointState.effort.size())
        {
            jointState.effort[index] = -jointState.effort[index];
        }
    }

    void HeadInvertKinematicsPlugin::invertMotorCommandValue(
        hightorque_msgs::msg::MotorControlCommand& command,
        std::size_t index) const
    {
        if (index < command.positions.size())
        {
            command.positions[index] = -command.positions[index];
        }
        if (index < command.velocities.size())
        {
            command.velocities[index] = -command.velocities[index];
        }
        if (index < command.torques.size())
        {
            command.torques[index] = -command.torques[index];
        }
    }

} // namespace hightorque

PLUGINLIB_EXPORT_CLASS(hightorque::HeadInvertKinematicsPlugin, hightorque::KinematicsPlugin)
