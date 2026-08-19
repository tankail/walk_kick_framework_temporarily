#ifndef MOTOR_MANAGER_NODE_H_
#define MOTOR_MANAGER_NODE_H_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "hardware/robot.hpp"
#include "hightorque_kinematics_core/hightorque_kinematics_plugin.hpp"
#include "motor/common_motor_group.h"
#include "motor/motor_group.h"
#include "motor/robot_motor_group.h"
#include "utils/common.h"

#include "hightorque_msgs/msg/motor_control_command.hpp"
#include "hightorque_msgs/msg/session_status.hpp"
#include "hightorque_msgs/srv/get_available_motors.hpp"
#include "hightorque_msgs/srv/release_control.hpp"
#include "hightorque_msgs/srv/request_control.hpp"
#include "hightorque_msgs/srv/reset_zero.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace hightorque
{

    struct ControlSession
    {
            std::string uuid;
            std::string nodeName;
            std::vector<int32_t> motorIds;
            std::vector<std::string> motorNames;
            uint8_t controlMode;
            uint8_t defaultBehavior;
            std::vector<float> defaultKp;
            std::vector<float> defaultKd;
            std::chrono::steady_clock::time_point lastCommandTime;
            int32_t timeoutMs;
            bool active;
    };

    class MotorManagerNode : public rclcpp::Node
    {
        public:
            MotorManagerNode();
            ~MotorManagerNode();

            void init();
            void spin();

        private:
            void initRobot();
            void initServices();
            void initTopics();
            void initTimers();

            void handleGetAvailableMotors(
                const std::shared_ptr<hightorque_msgs::srv::GetAvailableMotors::Request> request,
                std::shared_ptr<hightorque_msgs::srv::GetAvailableMotors::Response> response);

            void handleRequestControl(
                const std::shared_ptr<hightorque_msgs::srv::RequestControl::Request> request,
                std::shared_ptr<hightorque_msgs::srv::RequestControl::Response> response);

            void handleReleaseControl(
                const std::shared_ptr<hightorque_msgs::srv::ReleaseControl::Request> request,
                std::shared_ptr<hightorque_msgs::srv::ReleaseControl::Response> response);

            void handleResetZero(
                const std::shared_ptr<hightorque_msgs::srv::ResetZero::Request> request,
                std::shared_ptr<hightorque_msgs::srv::ResetZero::Response> response);

            void handleControlCommand(const hightorque_msgs::msg::MotorControlCommand::SharedPtr msg);

            void checkTimeout();
            void publishMotorStates();
            void publishSessionStatus();
            void printMotorOccupancy();

            bool isMotorAvailable(int32_t motorId);
            bool validateSession(const std::string& uuid);
            bool validateControlCommandMessage(const hightorque_msgs::msg::MotorControlCommand& command) const;
            bool buildCurrentJointState(sensor_msgs::msg::JointState& jointState, bool updateMotorHealth);
            void loadKinematicsPlugin();
            void applyControlCommand(const hightorque_msgs::msg::MotorControlCommand::SharedPtr msg);
            void releaseSessionsUsingMotor(int32_t motorId);

            void motorSendThread();
            void initializeMotorsToZeroTorque();

            std::shared_ptr<hightorque_robot::robot> robotPtr_;
            std::shared_ptr<RobotMotorGroup> robotMotorGroupPtr_;

            std::map<int32_t, std::pair<uint8_t, bool>> motorStatus_;
            std::map<int32_t, std::string> motorNames_;
            std::map<int32_t, std::string> motorToNode_;
            std::map<int32_t, int> motorRefCount_;
            std::map<int32_t, double> defaultKp_;
            std::map<int32_t, double> defaultKd_;

            std::unordered_map<std::string, ControlSession> sessions_;
            std::unordered_map<int32_t, std::string> motorToSession_;
            std::recursive_mutex sessionMutex_;

            std::thread sendThread_;
            std::atomic<bool> sendThreadRunning_;
            std::mutex sendMutex_;

            rclcpp::Service<hightorque_msgs::srv::GetAvailableMotors>::SharedPtr getMotorsService_;
            rclcpp::Service<hightorque_msgs::srv::RequestControl>::SharedPtr requestControlService_;
            rclcpp::Service<hightorque_msgs::srv::ReleaseControl>::SharedPtr releaseControlService_;
            rclcpp::Service<hightorque_msgs::srv::ResetZero>::SharedPtr resetZeroService_;

            rclcpp::Subscription<hightorque_msgs::msg::MotorControlCommand>::SharedPtr controlCommandSub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub_;
            rclcpp::Publisher<hightorque_msgs::msg::SessionStatus>::SharedPtr sessionStatusPub_;

            rclcpp::TimerBase::SharedPtr timeoutTimer_;
            rclcpp::TimerBase::SharedPtr statePubTimer_;
            rclcpp::TimerBase::SharedPtr sessionStatusTimer_;
            rclcpp::TimerBase::SharedPtr statusPrintTimer_;

            std::string robotParamFile_;
            std::string robotConfigPath_;
            std::string robotType_;
            std::string modelType_;
            std::string jointsConfigFile_;
            int defaultTimeoutMs_;

            std::vector<std::string> jointNames_;
            std::vector<int32_t> jointMotorIds_;
            std::string kinematicsPluginName_;
            std::unique_ptr<pluginlib::ClassLoader<hightorque::KinematicsPlugin>> kinematicsPluginLoader_;
            std::shared_ptr<hightorque::KinematicsPlugin> kinematicsPlugin_;
    };

} // namespace hightorque

#endif // MOTOR_MANAGER_NODE_H_
