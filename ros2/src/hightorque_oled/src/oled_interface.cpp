#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "hightorque_oled/sensor_actuator_status.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "hightorque_oled/ip_addr.hpp"
#include <thread>
#include <chrono>
#include "hightorque_power/msg/version_info.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>

#define _2PI_ 6.2831853

Sensor_actuator_status *status;

uint8_t imu_flag = 0;
float rpy[3];

int can_port_num;
int motor_num[8]={0, 0, 0, 0, 0, 0, 0, 0};
std::string oled_uart_dev_name;

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr data);
void motor_callback(const sensor_msgs::msg::JointState::SharedPtr data);
void battery_level_callback(const std_msgs::msg::UInt8::SharedPtr msg);
void fsm_state_callback(const std_msgs::msg::Int32::SharedPtr msg);
void version_callback(const hightorque_power::msg::VersionInfo::SharedPtr msg);

// Load ROS1 format YAML configuration
bool load_ros1_yaml_config(const std::string& yaml_file, int& can_port_num, int motor_num[8], std::shared_ptr<rclcpp::Node> node)
{
    try
    {
        YAML::Node config = YAML::LoadFile(yaml_file);

        if (!config["robot"] || !config["robot"]["CANboard"] || !config["robot"]["CANboard"]["No_1_CANboard"])
        {
            RCLCPP_ERROR(node->get_logger(), "Invalid YAML structure");
            return false;
        }

        YAML::Node canboard = config["robot"]["CANboard"]["No_1_CANboard"];

        // Get CAN port number
        if (canboard["CANport_num"])
        {
            can_port_num = canboard["CANport_num"].as<int>();
            RCLCPP_INFO(node->get_logger(), "Robot has %d CAN", can_port_num);
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "CANport_num not found in YAML");
            return false;
        }

        // Get motor numbers for each CAN port
        if (canboard["CANport"])
        {
            YAML::Node canports = canboard["CANport"];

            for (int i = 0; i < can_port_num; i++)
            {
                std::string port_name = "CANport_" + std::to_string(i + 1);

                if (canports[port_name])
                {
                    YAML::Node port = canports[port_name];

                    if (port["serial_id"] && port["motor_num"])
                    {
                        int serial_id = port["serial_id"].as<int>();
                        int motor_count = port["motor_num"].as<int>();

                        if (serial_id > 0 && serial_id <= 8)
                        {
                            motor_num[serial_id - 1] = motor_count;
                            RCLCPP_INFO(node->get_logger(), "can_%d has %d motor", i + 1, motor_count);
                        }
                        else
                        {
                            RCLCPP_ERROR(node->get_logger(), "Invalid serial_id: %d", serial_id);
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(node->get_logger(), "Missing serial_id or motor_num for %s", port_name.c_str());
                    }
                }
                else
                {
                    RCLCPP_ERROR(node->get_logger(), "CANport %s not found", port_name.c_str());
                }
            }
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "CANport section not found in YAML");
            return false;
        }

        return true;
    }
    catch (const YAML::Exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "YAML parsing error: %s", e.what());
        return false;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error loading YAML: %s", e.what());
        return false;
    }
}

void oled_mission(std::shared_ptr<rclcpp::Node> node)
{
    int serial_id = 0;
    rclcpp::Rate r(10.0);

    // Get config file path from ROS2 parameter or use default
    std::string config_file_path;
    node->declare_parameter("config_file", "");
    node->get_parameter("config_file", config_file_path);

    // Load ROS1 format YAML configuration
    if (!config_file_path.empty())
    {
        RCLCPP_INFO(node->get_logger(), "Loading config from: %s", config_file_path.c_str());
        if (!load_ros1_yaml_config(config_file_path, can_port_num, motor_num, node))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to load config file, using default parameters");
            can_port_num = 0;
        }
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "No config file specified, trying ROS2 parameters");

        // Fallback to ROS2 parameters if no config file provided
        node->declare_parameter("robot.CANboard.No_1_CANboard.CANport_num", 0);
        node->get_parameter("robot.CANboard.No_1_CANboard.CANport_num", can_port_num);

        if (can_port_num > 0)
        {
            RCLCPP_INFO(node->get_logger(), "Robot has %d CAN", can_port_num);
            for(int i = 0; i < can_port_num; i++)
            {
                std::string serial_id_param = "robot.CANboard.No_1_CANboard.CANport.CANport_" + std::to_string(i+1) + ".serial_id";
                std::string motor_num_param = "robot.CANboard.No_1_CANboard.CANport.CANport_" + std::to_string(i+1) + ".motor_num";

                // Declare parameters before getting them
                node->declare_parameter(serial_id_param, 0);
                node->declare_parameter(motor_num_param, 0);

                if (node->get_parameter(serial_id_param, serial_id) && serial_id > 0)
                {
                    if (node->get_parameter(motor_num_param, motor_num[serial_id-1]))
                    {
                        RCLCPP_INFO(node->get_logger(), "can_%d has %d motor", i+1, motor_num[serial_id-1]);
                    }
                    else
                    {
                        RCLCPP_ERROR(node->get_logger(), "Failed to get params motor_num");
                    }
                }
                else
                {
                    RCLCPP_ERROR(node->get_logger(), "Failed to get params serial_id");
                }
            }
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get params can_port_num or value is 0");
        }
    }

    node->declare_parameter("oled_uart_dev_name", "/dev/ttyACM");
    node->get_parameter("oled_uart_dev_name", oled_uart_dev_name);

    RCLCPP_INFO(node->get_logger(), "all_motor_num: %d", motor_num[0]+motor_num[1]+motor_num[2]+motor_num[3]+motor_num[4]+motor_num[5]+motor_num[6]+motor_num[7]);
    RCLCPP_INFO(node->get_logger(), "oled_uart_dev_name: %s", oled_uart_dev_name.c_str());

    // Create motor status object
    status = new Sensor_actuator_status(motor_num[0], motor_num[1], motor_num[2], motor_num[3], motor_num[4], motor_num[5], motor_num[6], motor_num[7], oled_uart_dev_name);

    // Subscribe to topics
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 1, imu_callback);
    auto motor_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "error_joint_states", 1, motor_callback);
    auto battery_level_sub = node->create_subscription<std_msgs::msg::UInt8>(
        "battery_level", 1, battery_level_callback);
    auto fsm_state_sub = node->create_subscription<std_msgs::msg::Int32>(
        "fsm_state", 1, fsm_state_callback);
    auto version_sub = node->create_subscription<hightorque_power::msg::VersionInfo>(
        "version", 10, version_callback);

    int i = 0;
    update_ip_addr();

    while(rclcpp::ok())
    {
        r.sleep();
        rclcpp::spin_some(node);
        i++;
        if(i>=10)
        {
            i = 0;
            update_ip_addr();
        }
        status->send_ip_addr(get_ip_data_u32_all(), 3);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(imu_flag)
        {
            status->send_imu_status(0, rpy);
        }
        imu_flag = 1;
    }
}

void version_callback(const hightorque_power::msg::VersionInfo::SharedPtr msg)
{
    status->send_version(msg->device_type, msg->version.c_str());
}

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr data)
{
    // static int cnt = 0;
    double roll, pitch, yaw;

    tf2::Quaternion q(
        data->orientation.x,
        data->orientation.y,
        data->orientation.z,
        data->orientation.w
    );

    tf2::Matrix3x3 rot_mat(q);
    rot_mat.getRPY(roll, pitch, yaw);

    float rpy[3];
    rpy[0] = roll;
    rpy[1] = pitch;
    rpy[2] = yaw;
    status->send_imu_status(1, rpy);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    imu_flag = 0;
}

void motor_callback(const sensor_msgs::msg::JointState::SharedPtr data)
{
    unsigned char motor_status[64] = {1,0,1,0,1,1,0,1,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0};
    int all_motor_num;
    all_motor_num = motor_num[0] + motor_num[1] + motor_num[2] + motor_num[3] + motor_num[4] + motor_num[5] + motor_num[6] + motor_num[7];
    for(int i = 0; i < all_motor_num; i++)
    {
        if(data->position[i] < -900.0)
        {
            motor_status[i] = 0;
        }
        else
        {
            motor_status[i] = 1;
        }
    }
    status->send_motor_status(motor_status);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void battery_level_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    status->send_battery_level(msg->data);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void fsm_state_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    status->send_fsm_state(msg->data);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
