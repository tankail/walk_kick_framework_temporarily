#ifndef HEAD_INVERT_KINEMATICS_PLUGIN_HPP_
#define HEAD_INVERT_KINEMATICS_PLUGIN_HPP_

#include <unordered_set>
#include <vector>

#include "hightorque_kinematics_core/hightorque_kinematics_plugin.hpp"

namespace hightorque
{

    class HeadInvertKinematicsPlugin : public KinematicsPlugin
    {
        public:
            bool configure(const KinematicsPluginConfig& config) override;

            bool forward(
                const sensor_msgs::msg::JointState& currentState,
                sensor_msgs::msg::JointState& publishedState) override;

            bool inverse(
                const sensor_msgs::msg::JointState& currentState,
                const hightorque_msgs::msg::MotorControlCommand& command,
                hightorque_msgs::msg::MotorControlCommand& inversedCommand) override;

            const KinematicsPluginConfig& config() const override;

        private:
            bool isHeadJoint(const std::string& jointName) const;
            void invertJointStateValue(sensor_msgs::msg::JointState& jointState, std::size_t index) const;
            void invertMotorCommandValue(hightorque_msgs::msg::MotorControlCommand& command, std::size_t index) const;

            KinematicsPluginConfig config_;
            std::vector<int32_t> invertedMotorIds_;
            std::unordered_set<std::string> headJointNames_;
    };

} // namespace hightorque

#endif // HEAD_INVERT_KINEMATICS_PLUGIN_HPP_
