#include "motor_manager_node.h"
#include "hightorque_msgs/msg/session_status.hpp"

#include <cmath>

namespace
{
    constexpr int32_t kDefaultResetZeroTimeoutMs = 10000;
    constexpr double kResetZeroPollRateHz = 10.0;
    constexpr float kResetZeroPositionTolerance = 0.1f;

    float getConfiguredGain(const std::map<int32_t, double>& gains, int32_t motorId)
    {
        auto it = gains.find(motorId);
        return it != gains.end() ? static_cast<float>(it->second) : 0.0f;
    }

    std::vector<float> resolveDefaultGains(const std::vector<int32_t>& motorIds,
                                           const std::vector<float>& requestedGains,
                                           const std::map<int32_t, double>& fallbackGains)
    {
        std::vector<float> resolved;
        resolved.reserve(motorIds.size());

        for (size_t i = 0; i < motorIds.size(); ++i)
        {
            if (i < requestedGains.size())
            {
                resolved.push_back(requestedGains[i]);
            }
            else
            {
                resolved.push_back(getConfiguredGain(fallbackGains, motorIds[i]));
            }
        }

        return resolved;
    }

    float getSessionGain(const std::vector<float>& gains, size_t index)
    {
        return index < gains.size() ? gains[index] : 0.0f;
    }

    float resolvePositionModeGain(const std::vector<float>& msgGains,
                                  size_t msgIndex,
                                  const std::vector<float>& sessionGains,
                                  size_t sessionIndex,
                                  const std::map<int32_t, double>& configuredGains,
                                  int32_t motorId)
    {
        if (msgIndex < msgGains.size())
        {
            return msgGains[msgIndex];
        }

        if (sessionIndex < sessionGains.size())
        {
            return sessionGains[sessionIndex];
        }

        return getConfiguredGain(configuredGains, motorId);
    }

    bool isValidCalibratedMotorState(const motor_back_t& state, double calibrationStartTimeSec)
    {
        if (state.position == 999.0f)
        {
            return false;
        }

        if (!std::isfinite(state.position))
        {
            return false;
        }

        if (state.time < calibrationStartTimeSec)
        {
            return false;
        }

        return state.fault == 0 && std::abs(state.position) <= kResetZeroPositionTolerance;
    }
}

namespace hightorque
{

    MotorManagerNode::MotorManagerNode()
        : Node("motor_manager_node"), defaultTimeoutMs_(5000), sendThreadRunning_(false)
    {
        RCLCPP_INFO(this->get_logger(), "MotorManagerNode constructor");
    }

    MotorManagerNode::~MotorManagerNode()
    {
        RCLCPP_INFO(this->get_logger(), "MotorManagerNode destructor");

        // 停止发送线程
        if (sendThreadRunning_)
        {
            sendThreadRunning_ = false;
            if (sendThread_.joinable())
            {
                sendThread_.join();
            }
            RCLCPP_INFO(this->get_logger(), "Motor send thread stopped");
        }

        // 关闭前广播空 SessionStatus，通知所有订阅者 UUID 全部失效
        if (sessionStatusPub_)
        {
            hightorque_msgs::msg::SessionStatus emptyMsg;
            emptyMsg.header.stamp = this->now();
            sessionStatusPub_->publish(emptyMsg);
        }

        // 释放所有会话
        std::lock_guard<std::recursive_mutex> lock(sessionMutex_);
        sessions_.clear();
        motorToSession_.clear();

        // 释放机器人资源
        robotMotorGroupPtr_.reset();
        robotPtr_.reset();
    }

    void MotorManagerNode::init()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MotorManagerNode...");

        // 声明参数
        this->declare_parameter<std::string>("robot_param_file", "");
        this->declare_parameter<std::string>("robot_config_path", "");
        this->declare_parameter<std::string>("robot_type", "");
        this->declare_parameter<std::string>("model_type", "pi_plus");
        this->declare_parameter<std::string>("joints_config_file", "");
        this->declare_parameter<int>("default_timeout_ms", 200);

        // 获取参数
        robotParamFile_ = this->get_parameter("robot_param_file").as_string();
        robotConfigPath_ = this->get_parameter("robot_config_path").as_string();
        robotType_ = this->get_parameter("robot_type").as_string();
        modelType_ = this->get_parameter("model_type").as_string();
        jointsConfigFile_ = this->get_parameter("joints_config_file").as_string();
        defaultTimeoutMs_ = this->get_parameter("default_timeout_ms").as_int();

        // 如果没有指定 joints_config_file，从 robot_config_path 构建
        if (jointsConfigFile_.empty() && !robotConfigPath_.empty())
        {
            jointsConfigFile_ = robotConfigPath_ + "/joints.yaml";
        }

        // 验证参数
        if (robotParamFile_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "robot_param_file not set!");
            throw std::runtime_error("Required parameter robot_param_file not set!");
        }

        RCLCPP_INFO(this->get_logger(), "Robot parameters:");
        RCLCPP_INFO(this->get_logger(), "  robot_param_file: %s", robotParamFile_.c_str());
        RCLCPP_INFO(this->get_logger(), "  robot_config_path: %s", robotConfigPath_.c_str());
        RCLCPP_INFO(this->get_logger(), "  robot_type: %s", robotType_.c_str());
        RCLCPP_INFO(this->get_logger(), "  model_type: %s", modelType_.c_str());
        RCLCPP_INFO(this->get_logger(), "  joints_config_file: %s", jointsConfigFile_.c_str());

        // 初始化各个模块
        initRobot();
        initServices();
        initTopics();
        initTimers();

        RCLCPP_INFO(this->get_logger(), "MotorManagerNode initialized successfully");

        // 启动发送线程
        sendThreadRunning_ = true;
        sendThread_ = std::thread(&MotorManagerNode::motorSendThread, this);
        RCLCPP_INFO(this->get_logger(), "Motor send thread started at 1000Hz");
    }

    void MotorManagerNode::initRobot()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing robot...");

        // 创建机器人实例
        robotPtr_ = std::make_shared<hightorque_robot::robot>(robotParamFile_);

        // 解析 joints.yaml 文件
        std::vector<std::string> jointNames;
        std::vector<int> mapIndex;
        std::vector<double> lower;
        std::vector<double> upper;
        std::vector<double> direction;
        std::vector<double> offset;
        std::vector<double> kp, kd;
        std::string configuredKinematicsPluginName;

        if (!jointsConfigFile_.empty())
        {
            try
            {
                RCLCPP_INFO(this->get_logger(), "Loading PD config from: %s", jointsConfigFile_.c_str());
                YAML::Node config = YAML::LoadFile(jointsConfigFile_);

                if (config["kinematics_plugin"])
                {
                    const YAML::Node& kinematicsPluginConfig = config["kinematics_plugin"];
                    if (kinematicsPluginConfig["name"])
                    {
                        configuredKinematicsPluginName = kinematicsPluginConfig["name"].as<std::string>();
                    }
                }

                if (config["joint_names"])
                {
                    jointNames = config["joint_names"].as<std::vector<std::string>>();
                }

                if (config["map_index"])
                {
                    mapIndex = config["map_index"].as<std::vector<int>>();
                }

                if (config["lower"])
                {
                    lower = config["lower"].as<std::vector<double>>();
                }

                if (config["upper"])
                {
                    upper = config["upper"].as<std::vector<double>>();
                }

                if (config["direction"])
                {
                    direction = config["direction"].as<std::vector<double>>();
                }
                else
                {
                    direction = std::vector<double>(jointNames.size(), 1.0);
                }

                if (config["urdf_offset"])
                {
                    offset = config["urdf_offset"].as<std::vector<double>>();
                }
                else
                {
                    offset = std::vector<double>(jointNames.size(), 0.0);
                }
                if (config["kp"])
                {
                    kp = config["kp"].as<std::vector<double>>();
                }
                else
                {
                    kp = std::vector<double>(jointNames.size(), 0.0);
                }
                if (config["kd"])
                {
                    kd = config["kd"].as<std::vector<double>>();
                }
                else
                {
                    kd = std::vector<double>(jointNames.size(), 0.0);
                }

                for (int i = 0; i < jointNames.size(); i++)
                {
                    defaultKp_[mapIndex[i]] = kp[i];
                    defaultKd_[mapIndex[i]] = kd[i];
                }

                RCLCPP_INFO(this->get_logger(), "Loaded %zu joints from PD config", jointNames.size());
            }
            catch (const YAML::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse PD config: %s", e.what());
                throw std::runtime_error("Failed to parse PD config file");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No PD config file specified, using default configuration");
        }

        RCLCPP_INFO(this->get_logger(), "Creating CommonMotorGroup");
        robotMotorGroupPtr_ = std::make_shared<CommonMotorGroup>(
            robotPtr_, mapIndex, jointNames, lower, upper, direction, offset);

        jointNames_ = jointNames;
        jointMotorIds_.clear();
        jointMotorIds_.reserve(mapIndex.size());
        for (int motorId : mapIndex)
        {
            jointMotorIds_.push_back(static_cast<int32_t>(motorId));
        }
        kinematicsPluginName_ = configuredKinematicsPluginName;
        loadKinematicsPlugin();

        auto motorState = robotMotorGroupPtr_->getMotorGroupStateByName("all");
        RCLCPP_INFO(this->get_logger(), "Motor group 'all' initialized with %zu motors ", motorState.size());

        // 初始化电机状态映射
        for (size_t i = 0; i < motorState.size(); i++)
        {
            const auto& motor = motorState[i];
            motorStatus_[motor.id] = {0, true}; // IDLE, healthy
            motorNames_[motor.id] = motor.name;
            RCLCPP_INFO(this->get_logger(), "  Motor [%zu]:id: [%d] %s", i, motor.id, jointNames[i].c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Robot initialized with %zu motors", motorStatus_.size());
    }

    void MotorManagerNode::initServices()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing services...");

        getMotorsService_ = this->create_service<hightorque_msgs::srv::GetAvailableMotors>(
            "get_available_motors",
            std::bind(&MotorManagerNode::handleGetAvailableMotors, this,
                      std::placeholders::_1, std::placeholders::_2));

        requestControlService_ = this->create_service<hightorque_msgs::srv::RequestControl>(
            "request_control",
            std::bind(&MotorManagerNode::handleRequestControl, this,
                      std::placeholders::_1, std::placeholders::_2));

        releaseControlService_ = this->create_service<hightorque_msgs::srv::ReleaseControl>(
            "release_control",
            std::bind(&MotorManagerNode::handleReleaseControl, this,
                      std::placeholders::_1, std::placeholders::_2));

        resetZeroService_ = this->create_service<hightorque_msgs::srv::ResetZero>(
            "reset_zero",
            std::bind(&MotorManagerNode::handleResetZero, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Services initialized");
    }

    void MotorManagerNode::initTopics()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing topics...");

        controlCommandSub_ = this->create_subscription<hightorque_msgs::msg::MotorControlCommand>(
            "control_command", 1000,
            std::bind(&MotorManagerNode::handleControlCommand, this, std::placeholders::_1));

        jointStatePub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 1000);

        sessionStatusPub_ = this->create_publisher<hightorque_msgs::msg::SessionStatus>(
            "session_status", 10);

        RCLCPP_INFO(this->get_logger(), "Topics initialized");
    }

    void MotorManagerNode::initTimers()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing timers...");

        // 超时检查定时器（100ms）
        timeoutTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotorManagerNode::checkTimeout, this));

        // 状态发布定时器（1ms，1000Hz）
        statePubTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&MotorManagerNode::publishMotorStates, this));

        // 会话状态广播定时器（1Hz，用于新上线节点快速获知有效UUID）
        sessionStatusTimer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MotorManagerNode::publishSessionStatus, this));

        // 电机占用情况打印定时器（30秒）
        statusPrintTimer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            std::bind(&MotorManagerNode::printMotorOccupancy, this));

        RCLCPP_INFO(this->get_logger(), "Timers initialized");
    }

    void MotorManagerNode::handleGetAvailableMotors(
        const std::shared_ptr<hightorque_msgs::srv::GetAvailableMotors::Request> request,
        std::shared_ptr<hightorque_msgs::srv::GetAvailableMotors::Response> response)
    {
        std::lock_guard<std::recursive_mutex> lock(sessionMutex_);

        // 如果提供了 node_name，则返回该节点已控制的电机 + 空闲电机
        // 否则只返回空闲电机
        std::string nodeName = request->node_name;

        for (const auto& [motor_id, status] : motorStatus_)
        {
            bool isAvailable = false;

            if (status.first == 0 && status.second)
            {
                // 空闲且健康的电机，对所有节点可用
                isAvailable = true;
            }
            else if (!nodeName.empty() && status.first == 1 && status.second)
            {
                // 电机被占用，检查是否被该节点控制
                // 通过 motorToNode_ 直接判断（motorToSession_ 是1:1映射会被覆盖，不可靠）
                auto nodeIt = motorToNode_.find(motor_id);
                if (nodeIt != motorToNode_.end() && nodeIt->second == nodeName)
                {
                    isAvailable = true;
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Motor :%d, Occupancy: %d,error: %d",
                            motor_id, status.first, status.second);
            }

            if (isAvailable)
            {
                response->motor_ids.push_back(motor_id);
                response->motor_names.push_back(motorNames_[motor_id]);
                response->motor_status.push_back(status.first);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Get available motors for node '%s': %zu motors",
                    nodeName.c_str(), response->motor_ids.size());
    }

    void MotorManagerNode::handleRequestControl(
        const std::shared_ptr<hightorque_msgs::srv::RequestControl::Request> request,
        std::shared_ptr<hightorque_msgs::srv::RequestControl::Response> response)
    {
        std::lock_guard<std::recursive_mutex> lock(sessionMutex_);

        // 验证节点名称
        if (request->node_name.empty())
        {
            response->success = false;
            response->message = "Node name is required";
            RCLCPP_WARN(this->get_logger(), "Request control failed: Node name is empty");
            return;
        }

        std::vector<std::shared_ptr<MotorBase>> motors;
        // 检查电机是否可用（考虑节点级别的引用计数）
        for (int32_t motorId : request->motor_ids)
        {
            auto motorName = motorNames_.find(motorId) != motorNames_.end() ? motorNames_[motorId] : "unknown";
            // 检查电机是否存在
            if (motorStatus_.find(motorId) == motorStatus_.end())
            {
                response->success = false;
                response->message = "Motor " + std::to_string(motorId) + " (" + motorName + ") does not exist";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                return;
            }
            // 检查电机是否处于故障状态
            if (!motorStatus_[motorId].second)
            {
                response->success = false;
                response->message = "Motor " + std::to_string(motorId) + " is in fault state";
                response->message = "Motor " + std::to_string(motorId) + " (" + motorName + ") is in fault state";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            motors.push_back(robotMotorGroupPtr_->getMotorByName(motorNames_[motorId]));

            // 检查电机是否被其他节点占用
            if (motorToNode_.find(motorId) != motorToNode_.end())
            {
                const std::string& occupyingNode = motorToNode_[motorId];
                if (occupyingNode != request->node_name)
                {
                    response->success = false;
                    response->message = "Motor " + std::to_string(motorId) +
                                        " is occupied by node: " + occupyingNode;
                    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                    return;
                }
                // 同一节点可以多次申请，引用计数+1
                // RCLCPP_INFO(this->get_logger(), "Motor %d already occupied by node %s, ref_count will increase",
                // motorId, request->node_name.c_str());
            }
        }

        // 生成UUID
        std::string uuid = utils::generateUUID();

        robotMotorGroupPtr_->registerMotorGroup(uuid, motors);
        RCLCPP_INFO(this->get_logger(), "Registered motor group with UUID: %s for node: %s ", uuid.c_str(), request->node_name.c_str());

        // 创建会话
        ControlSession session;
        session.uuid = uuid;
        session.nodeName = request->node_name;
        session.motorIds = request->motor_ids;
        session.controlMode = request->control_mode;
        session.defaultBehavior = request->default_behavior;
        session.defaultKp = resolveDefaultGains(request->motor_ids, request->default_kp, defaultKp_);
        session.defaultKd = resolveDefaultGains(request->motor_ids, request->default_kd, defaultKd_);
        session.timeoutMs = request->timeout_ms > 0 ? request->timeout_ms : defaultTimeoutMs_;
        session.lastCommandTime = std::chrono::steady_clock::now();
        session.active = true;

        // 填充 motorNames
        for (int32_t motor_id : request->motor_ids)
        {
            session.motorNames.push_back(motorNames_[motor_id]);
        }

        // 保存会话
        sessions_[uuid] = session;

        // 更新电机状态和引用计数
        for (int32_t motor_id : request->motor_ids)
        {
            // 更新引用计数
            if (motorRefCount_.find(motor_id) == motorRefCount_.end())
            {
                motorRefCount_[motor_id] = 0;
            }
            motorRefCount_[motor_id]++;

            // 标记电机为占用状态
            motorStatus_[motor_id] = {1, true}; // OCCUPIED
            motorToNode_[motor_id] = request->node_name;
            motorToSession_[motor_id] = uuid;

            // RCLCPP_INFO(this->get_logger(), "Motor %d: node=%s, ref_count=%d",
            //             motor_id, request->node_name.c_str(), motorRefCount_[motor_id]);
        }

        response->success = true;
        response->uuid = uuid;
        response->message = "Control granted";

        // 构建电机名称列表字符串
        std::string motorList;
        for (size_t i = 0; i < request->motor_ids.size(); ++i)
        {
            if (i > 0)
                motorList += ", ";
            motorList += motorNames_[request->motor_ids[i]];
        }

        // 获取 defaultBehavior 字符串
        std::string behaviorStr;
        switch (request->default_behavior)
        {
            case hightorque_msgs::srv::RequestControl::Request::KEEP_MODE:
                behaviorStr = "KEEP_MODE";
                break;
            case hightorque_msgs::srv::RequestControl::Request::DAMPING_MODE:
                behaviorStr = "DAMPING_MODE";
                break;
            case hightorque_msgs::srv::RequestControl::Request::ZERO_TORQUE_MODE:
                behaviorStr = "ZERO_TORQUE_MODE";
                break;
            default:
                behaviorStr = "UNKNOWN";
                break;
        }

        RCLCPP_INFO(this->get_logger(), "Control granted: node=%s, UUID=%s, motors=[%s], defaultBehavior=%s",
                    request->node_name.c_str(), uuid.substr(0, 8).c_str(), motorList.c_str(), behaviorStr.c_str());

        publishSessionStatus();
    }

    void MotorManagerNode::handleReleaseControl(
        const std::shared_ptr<hightorque_msgs::srv::ReleaseControl::Request> request,
        std::shared_ptr<hightorque_msgs::srv::ReleaseControl::Response> response)
    {
        std::lock_guard<std::recursive_mutex> lock(sessionMutex_);

        // 验证 node_name 是否提供
        if (request->node_name.empty())
        {
            response->success = false;
            response->message = "Node name is required";
            RCLCPP_WARN(this->get_logger(), "Release control failed: Node name is empty");
            return;
        }

        auto it = sessions_.find(request->uuid);
        if (it == sessions_.end())
        {
            response->success = false;
            response->message = "Invalid UUID";
            RCLCPP_WARN(this->get_logger(), "[%s]Release control failed: Invalid UUID :[%s] ", request->node_name.c_str(), request->uuid.c_str());
            return;
        }

        const std::string nodeName = it->second.nodeName;          // 值拷贝，避免引用失效
        const std::vector<int32_t> motorIds = it->second.motorIds; // 值拷贝

        // 验证节点名称是否匹配
        if (request->node_name != nodeName)
        {
            response->success = false;
            response->message = "Node name mismatch: only the requesting node can release its own control";
            RCLCPP_WARN(this->get_logger(),
                        "Release control failed: Node name mismatch (request=%s, session=%s)",
                        request->node_name.c_str(), nodeName.c_str());
            return;
        }

        // 减少引用计数，只有引用计数为0时才执行释放行为
        std::vector<int32_t> motorsToRelease; // 引用计数为0的电机

        for (int32_t motor_id : motorIds)
        {
            // 减少引用计数
            if (motorRefCount_.find(motor_id) != motorRefCount_.end())
            {
                motorRefCount_[motor_id]--;

                // 如果引用计数为0，标记为需要释放
                if (motorRefCount_[motor_id] <= 0)
                {
                    motorsToRelease.push_back(motor_id);
                    motorRefCount_.erase(motor_id);
                    motorToNode_.erase(motor_id);
                    motorToSession_.erase(motor_id);
                    motorStatus_[motor_id] = {0, true}; // IDLE
                }
            }

            robotMotorGroupPtr_->unregisterMotorGroup(request->uuid);
        }

        // 只对引用计数为0的电机执行释放行为
        if (!motorsToRelease.empty() && robotMotorGroupPtr_)
        {

            for (int32_t motorId : motorsToRelease)
            {

                auto motor = robotMotorGroupPtr_->getMotorByName(motorNames_[motorId]);
                switch (request->release_mode)
                {
                    case 0: // KEEP_STATE - 保持原状（使用当前kp/kd保持位置）
                        {
                            // 获取当前电机状态
                            motor->keepMotorLastCmd();
                            break;
                        }

                    case 1: // DAMPING_MODE - 阻尼模式（小kp/kd，可手动移动）
                        {
                            motor->protectMotor();
                            break;
                        }

                    case 2: // NO_CONTROL - 不控制（零力矩，完全放松）
                        {
                            MotorControlCmd cmd;
                            cmd.position = 0.0f;
                            cmd.velocity = 0.0f;
                            cmd.torque = 0.0f;
                            cmd.kp = 0.0f; // 零刚度
                            cmd.kd = 0.0f; // 零阻尼
                            cmd.type = MotorControlType::POS_VEL_TQE_KP_KD;
                            motor->setMotor(cmd);
                            break;
                        }

                    default:
                        RCLCPP_WARN(this->get_logger(), "Unknown release_mode: %d, using KEEP_STATE",
                                    request->release_mode);
                        motor->protectMotor();
                        break;
                }
            }
        }

        // 删除会话
        sessions_.erase(it);
        // 获取 release_mode 字符串
        std::string releaseModeStr;

        response->success = true;
        if (motorsToRelease.empty())
        {
            response->message = request->uuid + " Control released for " + std::to_string(motorIds.size()) + " motors, but " + request->node_name + " still have control of these motors.";
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
            releaseModeStr = "Nothing needs to be done.";
        }
        else
        {
            response->message = "Control released";
            switch (request->release_mode)
            {
                case hightorque_msgs::srv::ReleaseControl::Request::KEEP_MODE:
                    releaseModeStr = "KEEP_MODE";
                    break;
                case hightorque_msgs::srv::ReleaseControl::Request::DAMPING_MODE:
                    releaseModeStr = "DAMPING_MODE";
                    break;
                case hightorque_msgs::srv::ReleaseControl::Request::ZERO_TORQUE_MODE:
                    releaseModeStr = "ZERO_TORQUE_MODE";
                    break;
                default:
                    releaseModeStr = "UNKNOWN";
                    break;
            }
        }

        // 构建释放的电机名称列表字符串
        std::string motorList;
        for (size_t i = 0; i < motorsToRelease.size(); ++i)
        {
            if (i > 0)
                motorList += ", ";
            motorList += motorNames_[motorsToRelease[i]];
        }

        RCLCPP_INFO(this->get_logger(), "Control released: node=%s, UUID=%s, motors=[%s], releaseMode=%s",
                    nodeName.c_str(), request->uuid.substr(0, 8).c_str(), motorList.c_str(), releaseModeStr.c_str());

        publishSessionStatus();
    }

    void MotorManagerNode::handleResetZero(
        const std::shared_ptr<hightorque_msgs::srv::ResetZero::Request> request,
        std::shared_ptr<hightorque_msgs::srv::ResetZero::Response> response)
    {
        std::lock_guard<std::recursive_mutex> sessionLock(sessionMutex_);

        if (!robotPtr_ || !robotMotorGroupPtr_)
        {
            response->success = false;
            response->status = hightorque_msgs::srv::ResetZero::Response::STATUS_FAILED;
            response->message = "Robot is not initialized";
            RCLCPP_ERROR(this->get_logger(), "Reset zero failed: robot is not initialized");
            return;
        }

        if (!sessions_.empty())
        {
            response->success = false;
            response->status = hightorque_msgs::srv::ResetZero::Response::STATUS_FAILED;
            response->message = "Cannot reset zero while control sessions are active";
            RCLCPP_WARN(this->get_logger(), "Reset zero rejected: %zu active session(s)", sessions_.size());
            return;
        }

        initializeMotorsToZeroTorque();
        robotMotorGroupPtr_->sendMotors();
        robotPtr_->set_reset_zero();

        const int32_t timeoutMs = request->timeout_ms > 0 ? request->timeout_ms : kDefaultResetZeroTimeoutMs;
        const auto calibrationStartSteady = std::chrono::steady_clock::now();
        const auto calibrationStartSystem = std::chrono::system_clock::now();
        const double calibrationStartTimeSec =
            std::chrono::duration<double>(calibrationStartSystem.time_since_epoch()).count();

        std::lock_guard<std::mutex> sendLock(sendMutex_);

        RCLCPP_INFO(this->get_logger(), "Reset zero started with timeout %d ms", timeoutMs);

        rclcpp::Rate rate(kResetZeroPollRateHz);
        std::vector<int32_t> pendingMotorIds;

        while (rclcpp::ok())
        {
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       std::chrono::steady_clock::now() - calibrationStartSteady)
                                       .count();
            if (elapsedMs > timeoutMs)
            {
                initializeMotorsToZeroTorque();
                robotMotorGroupPtr_->sendMotors();

                std::string pendingList;
                for (size_t i = 0; i < pendingMotorIds.size(); ++i)
                {
                    if (i > 0)
                    {
                        pendingList += ", ";
                    }
                    pendingList += std::to_string(pendingMotorIds[i]);
                }

                response->success = false;
                response->status = hightorque_msgs::srv::ResetZero::Response::STATUS_TIMEOUT;
                response->message = pendingList.empty()
                                        ? "Reset zero timed out"
                                        : "Reset zero timed out, pending motors: " + pendingList;

                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            robotPtr_->send_get_motor_state_cmd();

            pendingMotorIds.clear();
            for (motor* rawMotor : robotPtr_->Motors)
            {
                if (rawMotor == nullptr)
                {
                    continue;
                }

                const motor_back_t state = *rawMotor->get_current_motor_state();
                if (!isValidCalibratedMotorState(state, calibrationStartTimeSec))
                {
                    pendingMotorIds.push_back(static_cast<int32_t>(state.ID));
                }
            }

            initializeMotorsToZeroTorque();
            robotMotorGroupPtr_->sendMotors();

            if (pendingMotorIds.empty())
            {
                response->success = true;
                response->status = hightorque_msgs::srv::ResetZero::Response::STATUS_SUCCEEDED;
                response->message = "Reset zero completed successfully";
                RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            rate.sleep();
        }

        initializeMotorsToZeroTorque();
        robotMotorGroupPtr_->sendMotors();

        response->success = false;
        response->status = hightorque_msgs::srv::ResetZero::Response::STATUS_FAILED;
        response->message = "Reset zero interrupted because ROS is shutting down";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    }

    void MotorManagerNode::handleControlCommand(
        const hightorque_msgs::msg::MotorControlCommand::SharedPtr msg)
    {
        std::lock_guard<std::recursive_mutex> lock(sessionMutex_);

        if (!validateSession(msg->uuid))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid control command: UUID not found");
            return;
        }

        hightorque_msgs::msg::MotorControlCommand commandToApply = *msg;
        if (kinematicsPlugin_)
        {
            sensor_msgs::msg::JointState currentState;
            if (!buildCurrentJointState(currentState, false))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to build current joint state for inverse kinematics");
                return;
            }

            hightorque_msgs::msg::MotorControlCommand inversedCommand;
            try
            {
                if (!kinematicsPlugin_->inverse(currentState, *msg, inversedCommand))
                {
                    RCLCPP_WARN(this->get_logger(), "Kinematics plugin inverse() returned false, dropping control command");
                    return;
                }
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Kinematics plugin inverse() threw an exception: %s", e.what());
                return;
            }

            inversedCommand.uuid = msg->uuid;
            commandToApply = std::move(inversedCommand);
        }

        if (!validateControlCommandMessage(commandToApply))
        {
            return;
        }

        sessions_[msg->uuid].lastCommandTime = std::chrono::steady_clock::now();
        applyControlCommand(std::make_shared<hightorque_msgs::msg::MotorControlCommand>(std::move(commandToApply)));
    }

    void MotorManagerNode::checkTimeout()
    {
        std::lock_guard<std::recursive_mutex> lock(sessionMutex_);
        auto now = std::chrono::steady_clock::now();

        std::vector<std::string> expired_sessions;

        for (auto& [uuid, session] : sessions_)
        {
            if (!session.active)
                continue;

            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now - session.lastCommandTime)
                               .count();

            if (elapsed > session.timeoutMs)
            {
                RCLCPP_WARN(this->get_logger(), "Session timeout: UUID=%s", uuid.c_str());
                expired_sessions.push_back(uuid);
            }
        }

        // 释放超时会话
        for (const auto& uuid : expired_sessions)
        {
            auto behavior = sessions_[uuid].defaultBehavior;
            for (int32_t motor_id : sessions_[uuid].motorIds)
            {
                // 减少引用计数
                if (motorRefCount_.find(motor_id) != motorRefCount_.end())
                {
                    motorRefCount_[motor_id]--;

                    // RCLCPP_INFO(this->get_logger(),
                    //             "Timeout: Motor %d ref_count decreased to %d",
                    // motor_id, motorRefCount_[motor_id]);

                    // 如果引用计数为0，清理相关映射
                    if (motorRefCount_[motor_id] <= 0)
                    {
                        motorRefCount_.erase(motor_id);
                        motorToNode_.erase(motor_id);
                        motorToSession_.erase(motor_id);
                        motorStatus_[motor_id] = {0, true}; // IDLE
                    }
                }
                else
                {
                    // 如果引用计数不存在，直接设为IDLE
                    motorStatus_[motor_id] = {0, true}; // IDLE
                    motorToSession_.erase(motor_id);
                }

                auto motor = robotMotorGroupPtr_->getMotorByName(motorNames_[motor_id]);
                switch (behavior)
                {
                    case hightorque_msgs::srv::RequestControl::Request::KEEP_MODE:
                        {
                            motor->keepMotorLastCmd();
                            // RCLCPP_INFO(this->get_logger(), "Motor %d: KEEP_MODE mode (timeout)", motor_id);
                            break;
                        }
                    case hightorque_msgs::srv::RequestControl::Request::ZERO_TORQUE_MODE:
                        {
                            MotorControlCmd cmd;
                            cmd.position = 0.0f;
                            cmd.velocity = 0.0f;
                            cmd.torque = 0.0f;
                            cmd.kp = 0.0f;
                            cmd.kd = 0.0f;
                            cmd.type = MotorControlType::POS_VEL_TQE_KP_KD;
                            motor->setMotor(cmd);
                            // RCLCPP_INFO(this->get_logger(), "Motor %d: ZERO_TORQUE_MODE mode (timeout)", motor_id);
                            break;
                        }
                    case hightorque_msgs::srv::RequestControl::Request::DAMPING_MODE:
                        {
                            motor->protectMotor();
                            // RCLCPP_INFO(this->get_logger(), "Motor %d: DAMPING_MODE mode (timeout)", motor_id);
                            break;
                        }
                    default:
                        {
                            motor->protectMotor();
                            // RCLCPP_WARN(this->get_logger(), "Motor %d: unknown defaultBehavior=%d, fallback to DAMPING (timeout)", motor_id, behavior);
                            break;
                        }
                }
            }

            // 注销电机组
            if (robotMotorGroupPtr_)
            {
                robotMotorGroupPtr_->unregisterMotorGroup(uuid);
            }

            sessions_.erase(uuid);

            RCLCPP_WARN(this->get_logger(),
                        "Session %s released due to timeout", uuid.c_str());
        }

        if (!expired_sessions.empty())
        {
            publishSessionStatus();
        }
    }

    void MotorManagerNode::publishMotorStates()
    {
        sensor_msgs::msg::JointState jointStateMsg;
        if (!buildCurrentJointState(jointStateMsg, true))
        {
            return;
        }

        if (kinematicsPlugin_)
        {
            sensor_msgs::msg::JointState publishedState;
            try
            {
                if (kinematicsPlugin_->forward(jointStateMsg, publishedState))
                {
                    jointStatePub_->publish(publishedState);
                    return;
                }

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Kinematics plugin forward() returned false, publishing raw joint state");
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Kinematics plugin forward() threw an exception: %s", e.what());
            }
        }

        jointStatePub_->publish(jointStateMsg);
    }

    void MotorManagerNode::publishSessionStatus()
    {
        if (!sessionStatusPub_)
            return;

        hightorque_msgs::msg::SessionStatus msg;
        msg.header.stamp = this->now();

        {
            std::lock_guard<std::recursive_mutex> lock(sessionMutex_);
            for (const auto& [uuid, session] : sessions_)
            {
                msg.active_uuids.push_back(uuid);
                msg.node_names.push_back(session.nodeName);
                msg.motor_counts.push_back(static_cast<int32_t>(session.motorIds.size()));
            }
        }

        sessionStatusPub_->publish(msg);
    }

    bool MotorManagerNode::isMotorAvailable(int32_t motor_id)
    {
        auto it = motorStatus_.find(motor_id);
        if (it == motorStatus_.end())
            return false;
        return it->second.first == 0 && it->second.second; // IDLE and healthy
    }

    bool MotorManagerNode::validateSession(const std::string& uuid)
    {
        return sessions_.find(uuid) != sessions_.end();
    }

    bool MotorManagerNode::validateControlCommandMessage(
        const hightorque_msgs::msg::MotorControlCommand& command) const
    {
        const size_t motorCount = command.motor_ids.size();
        if (motorCount == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid control command: motor_ids is empty");
            return false;
        }

        if (!command.positions.empty() && command.positions.size() != motorCount)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid control command: positions size (%zu) does not match motor_ids size (%zu)",
                command.positions.size(), motorCount);
            return false;
        }

        if (!command.velocities.empty() && command.velocities.size() != motorCount)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid control command: velocities size (%zu) must be empty or match motor_ids size (%zu)",
                command.velocities.size(), motorCount);
            return false;
        }

        if (!command.kp.empty() && command.kp.size() != motorCount)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid control command: kp size (%zu) must be empty or match motor_ids size (%zu)",
                command.kp.size(), motorCount);
            return false;
        }

        if (!command.kd.empty() && command.kd.size() != motorCount)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid control command: kd size (%zu) must be empty or match motor_ids size (%zu)",
                command.kd.size(), motorCount);
            return false;
        }

        if (!command.torques.empty() && command.torques.size() != motorCount)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid control command: torques size (%zu) must be empty or match motor_ids size (%zu)",
                command.torques.size(), motorCount);
            return false;
        }

        return true;
    }

    bool MotorManagerNode::buildCurrentJointState(sensor_msgs::msg::JointState& jointState, bool updateMotorHealth)
    {
        if (!robotMotorGroupPtr_)
        {
            return false;
        }

        const auto motorStates = robotMotorGroupPtr_->getMotorGroupStateByName("all");
        const auto now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

        jointState.header.stamp = this->now();
        jointState.header.frame_id = "";

        for (const auto& state : motorStates)
        {
            jointState.name.push_back(state.name);
            if (now - state.time > 1.0)
            {
                jointState.position.push_back(-1000.0);
                jointState.velocity.push_back(0.0);
                jointState.effort.push_back(0.0);
                auto motor = robotMotorGroupPtr_->getMotorByName(state.name);
                if (motor)
                    motor->setMotor(MotorControlCmd());

                if (updateMotorHealth)
                {
                    auto& motorState = motorStatus_[state.id];
                    if (motorState.second)
                    {
                        motorState.second = false;
                        releaseSessionsUsingMotor(state.id);
                    }
                }
                continue;
            }

            if (updateMotorHealth)
            {
                auto& motorState = motorStatus_[state.id];
                if (!motorState.second)
                {
                    motorState.second = true;
                    motorState.first = 0;
                }
            }

            jointState.position.push_back(state.position);
            jointState.velocity.push_back(state.velocity);
            jointState.effort.push_back(state.torque);
        }

        return true;
    }

    void MotorManagerNode::loadKinematicsPlugin()
    {
        kinematicsPlugin_.reset();
        kinematicsPluginLoader_.reset();

        if (kinematicsPluginName_.empty())
        {
            robotMotorGroupPtr_->setSupportsKinematics(false);
            RCLCPP_INFO(this->get_logger(), "No kinematics plugin configured, using raw midware pipeline");
            return;
        }

        try
        {
            kinematicsPluginLoader_ = std::make_unique<pluginlib::ClassLoader<hightorque::KinematicsPlugin>>(
                "hightorque_kinematics_core", "hightorque::KinematicsPlugin");
            kinematicsPlugin_ = kinematicsPluginLoader_->createSharedInstance(kinematicsPluginName_);

            hightorque::KinematicsPluginConfig config;
            config.jointNames = jointNames_;
            config.motorIds = jointMotorIds_;
            if (!kinematicsPlugin_->configure(config))
            {
                RCLCPP_ERROR(this->get_logger(), "Kinematics plugin '%s' configure() returned false, disabling plugin", kinematicsPluginName_.c_str());
                kinematicsPlugin_.reset();
                kinematicsPluginLoader_.reset();
                robotMotorGroupPtr_->setSupportsKinematics(false);
                return;
            }

            robotMotorGroupPtr_->setSupportsKinematics(true);
            RCLCPP_INFO(this->get_logger(), "Loaded kinematics plugin: %s", kinematicsPluginName_.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load kinematics plugin '%s': %s", kinematicsPluginName_.c_str(), e.what());
            kinematicsPlugin_.reset();
            kinematicsPluginLoader_.reset();
            robotMotorGroupPtr_->setSupportsKinematics(false);
        }
    }

    void MotorManagerNode::releaseSessionsUsingMotor(int32_t motorId)
    {
        std::lock_guard<std::recursive_mutex> lock(sessionMutex_);

        // 找到所有包含该电机的 session
        std::vector<std::string> uuidsToRelease;
        for (const auto& [uuid, session] : sessions_)
        {
            for (int32_t id : session.motorIds)
            {
                if (id == motorId)
                {
                    uuidsToRelease.push_back(uuid);
                    break;
                }
            }
        }

        for (const auto& uuid : uuidsToRelease)
        {
            auto it = sessions_.find(uuid);
            if (it == sessions_.end())
                continue;

            const auto& session = it->second;
            RCLCPP_WARN(this->get_logger(),
                        "Force releasing session [%s] (node: %s) due to motor [%d] fault",
                        uuid.c_str(), session.nodeName.c_str(), motorId);

            // 释放该 session 中所有电机
            for (int32_t id : session.motorIds)
            {
                auto refIt = motorRefCount_.find(id);
                if (refIt != motorRefCount_.end())
                {
                    refIt->second--;
                    if (refIt->second <= 0)
                    {
                        motorRefCount_.erase(refIt);
                        motorToNode_.erase(id);
                        motorToSession_.erase(id);
                        // 故障电机保持 ERROR 状态，其他电机恢复 IDLE
                        if (id != motorId)
                        {
                            motorStatus_[id] = {0, true}; // IDLE
                        }
                    }
                }
                else
                {
                    motorToNode_.erase(id);
                    motorToSession_.erase(id);
                    if (id != motorId)
                    {
                        motorStatus_[id] = {0, true}; // IDLE
                    }
                }
            }

            sessions_.erase(it);
        }
    }

    void MotorManagerNode::applyControlCommand(
        const hightorque_msgs::msg::MotorControlCommand::SharedPtr msg)
    {
        if (!robotMotorGroupPtr_)
            return;

        const auto& session = sessions_[msg->uuid];
        const auto& allowedIds = session.motorIds;
        const bool isPositionMode = session.controlMode == hightorque_msgs::srv::RequestControl::Request::POSITION_MODE;
        const bool isTorqueMode = session.controlMode == hightorque_msgs::srv::RequestControl::Request::TORQUE_MODE;
        const bool isMixedMode = session.controlMode == hightorque_msgs::srv::RequestControl::Request::MIXED_MODE;

        if (isPositionMode && msg->positions.size() < msg->motor_ids.size())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "position mode, but positions size %zu != motor id size %zu",
                                 msg->positions.size(), msg->motor_ids.size());
            return;
        }

        if (isTorqueMode && msg->torques.size() < msg->motor_ids.size())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "torque mode, but torques size %zu != motor id size %zu",
                                 msg->torques.size(), msg->motor_ids.size());
            return;
        }

        if (isMixedMode)
        {
            if (msg->positions.size() < msg->motor_ids.size() ||
                msg->torques.size() < msg->motor_ids.size() ||
                msg->kp.size() < msg->motor_ids.size() ||
                msg->kd.size() < msg->motor_ids.size())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "mixed mode requires positions, torques, kp, kd sizes to match motor_ids size %zu (got p=%zu, t=%zu, kp=%zu, kd=%zu)",
                                     msg->motor_ids.size(), msg->positions.size(), msg->torques.size(), msg->kp.size(), msg->kd.size());
                return;
            }
        }

        for (size_t i = 0; i < msg->motor_ids.size(); i++)
        {
            int32_t motorId = msg->motor_ids[i];

            // 检查该电机是否在此UUID的控制范围内
            auto it = std::find(allowedIds.begin(), allowedIds.end(), motorId);
            if (it == allowedIds.end())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Motor %d is not in the control scope of session [%s], ignoring command",
                                     motorId, msg->uuid.c_str());

                continue;
            }

            auto motor = robotMotorGroupPtr_->getMotorByName(motorNames_[motorId]);
            if (!motor)
                continue;

            const auto sessionIndex = static_cast<size_t>(std::distance(allowedIds.begin(), it));

            MotorControlCmd cmd;
            cmd.velocity = i < msg->velocities.size() ? msg->velocities[i] : 0.0f;
            cmd.type = MotorControlType::POS_VEL_TQE_KP_KD;

            if (isPositionMode)
            {
                cmd.position = msg->positions[i];
                cmd.torque = 0.0f;
                cmd.kp = resolvePositionModeGain(msg->kp, i, session.defaultKp, sessionIndex, defaultKp_, motorId);
                cmd.kd = resolvePositionModeGain(msg->kd, i, session.defaultKd, sessionIndex, defaultKd_, motorId);
            }
            else if (isTorqueMode)
            {
                cmd.position = 0.0f;
                cmd.torque = msg->torques[i];
                cmd.kp = 0.0f;
                cmd.kd = 0.0f;
            }
            else if (isMixedMode)
            {
                cmd.position = msg->positions[i];
                cmd.torque = msg->torques[i];
                cmd.kp = msg->kp[i];
                cmd.kd = msg->kd[i];
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "unknown control mode %u for session [%s]",
                                     session.controlMode, msg->uuid.c_str());
                continue;
            }

            motor->setMotor(cmd);
        }
    }

    void MotorManagerNode::spin()
    {
        RCLCPP_INFO(this->get_logger(), "MotorManagerNode spinning...");
    }

    void MotorManagerNode::initializeMotorsToZeroTorque()
    {
        /**
         * 初始化所有电机为零力矩状态
         * 位置=0, kp=0, kd=0 (完全放松)
         */

        if (!robotMotorGroupPtr_ || !robotPtr_)
        {
            RCLCPP_ERROR(this->get_logger(), "Robot not initialized, cannot initialize motors");
            return;
        }

        auto size = robotMotorGroupPtr_->getMotorGroupSizeByName("all");
        std::vector<MotorControlCmd> zeroTorqueCmds(size, MotorControlCmd{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0, 0.0, 0.0, 0.0, MotorControlType::POS_VEL_TQE_KP_KD});

        // 发送到硬件
        robotMotorGroupPtr_->setMotorGroupByName("all", zeroTorqueCmds);
    }

    void MotorManagerNode::printMotorOccupancy()
    {
        std::lock_guard<std::recursive_mutex> lock(sessionMutex_);

        RCLCPP_INFO(this->get_logger(), "========== Motor Occupancy Status ==========");

        if (sessions_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "No active control sessions");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Active sessions: %zu", sessions_.size());

            for (const auto& [uuid, session] : sessions_)
            {
                std::string motorList;
                for (size_t i = 0; i < session.motorIds.size(); ++i)
                {
                    if (i > 0)
                        motorList += ", ";
                    motorList += std::to_string(session.motorIds[i]);
                    if (i < session.motorNames.size())
                    {
                        motorList += "(" + session.motorNames[i] + ")";
                    }
                }

                RCLCPP_INFO(this->get_logger(), "  Node: %s | UUID: %s | Default Behavior %s | Control Mode: %s | Motors: [%s]",
                            session.nodeName.c_str(),
                            uuid.substr(0, 8).c_str(), // 只显示UUID前8位
                            session.defaultBehavior == hightorque_msgs::srv::RequestControl::Request::KEEP_MODE ? "KEEP_MODE" : session.defaultBehavior == hightorque_msgs::srv::RequestControl::Request::DAMPING_MODE   ? "DAMPING_MODE"
                                                                                                                            : session.defaultBehavior == hightorque_msgs::srv::RequestControl::Request::ZERO_TORQUE_MODE ? "ZERO_TORQUE_MODE"
                                                                                                                                                                                                                         : "UNKNOWN",
                            session.controlMode == hightorque_msgs::srv::RequestControl::Request::POSITION_MODE ? "POSITION_CONTROL" : session.controlMode == hightorque_msgs::srv::RequestControl::Request::TORQUE_MODE ? "TORQUE_MODE"
                                                                                                                                                                                                                         : "UNKNOWN",
                            motorList.c_str());
            }
        }

        // 统计空闲电机
        int idleCount = 0;
        int occupiedCount = 0;
        int faultCount = 0;
        for (const auto& [motor_id, status] : motorStatus_)
        {
            if (!status.second)
                faultCount++;
            else if (status.first == 0)
                idleCount++;
            else
                occupiedCount++;
        }

        RCLCPP_INFO(this->get_logger(), "Motor status: %d idle, %d occupied, %d fault",
                    idleCount, occupiedCount, faultCount);
        RCLCPP_INFO(this->get_logger(), "============================================");
    }

    void MotorManagerNode::motorSendThread()
    {
        /**
         * 电机发送线程，以1000Hz频率发送电机命令
         */
        RCLCPP_INFO(this->get_logger(), "Motor send thread started");

        // 线程开始时，初始化所有电机为零力矩状态
        initializeMotorsToZeroTorque();

        // 1000Hz = 1ms 周期

        rclcpp::Rate rate(1000);
        while (sendThreadRunning_)
        {

            {
                std::lock_guard<std::mutex> lock(sendMutex_);

                // 发送电机命令到硬件
                if (robotMotorGroupPtr_)
                {
                    robotMotorGroupPtr_->sendMotors();
                }
            }

            rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "Motor send thread exiting");
    }

} // namespace hightorque
