#ifndef HIGHTORQUE_KINEMATICS_CORE__HIGHTORQUE_KINEMATICS_PLUGIN_HPP_
#define HIGHTORQUE_KINEMATICS_CORE__HIGHTORQUE_KINEMATICS_PLUGIN_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "hightorque_msgs/msg/motor_control_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace hightorque
{

    struct KinematicsPluginConfig
    {
            std::vector<std::string> jointNames;
            std::vector<int32_t> motorIds;
    };

    class KinematicsPlugin
    {
        public:
            virtual ~KinematicsPlugin() = default;

            virtual bool configure(const KinematicsPluginConfig& config) = 0;

            virtual bool forward(
                const sensor_msgs::msg::JointState& currentState,
                sensor_msgs::msg::JointState& publishedState) = 0;

            virtual bool inverse(
                const sensor_msgs::msg::JointState& currentState,
                const hightorque_msgs::msg::MotorControlCommand& command,
                hightorque_msgs::msg::MotorControlCommand& inversedCommand) = 0;

            virtual const KinematicsPluginConfig& config() const = 0;
    };

} // namespace hightorque

#endif // HIGHTORQUE_KINEMATICS_CORE__HIGHTORQUE_KINEMATICS_PLUGIN_HPP_
