#ifndef NO_OP_KINEMATICS_PLUGIN_HPP_
#define NO_OP_KINEMATICS_PLUGIN_HPP_

#include "hightorque_kinematics_core/hightorque_kinematics_plugin.hpp"

namespace hightorque
{

    class NoOpKinematicsPlugin : public KinematicsPlugin
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
            KinematicsPluginConfig config_;
    };

} // namespace hightorque

#endif // NO_OP_KINEMATICS_PLUGIN_HPP_
