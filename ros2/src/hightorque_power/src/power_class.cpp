#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <hightorque_power/msg/power_switch.hpp>
#include "hightorque_power/msg/version_info.hpp"
#include "hightorque_power/livelybot_can_driver.hpp"
#include <ctime>
#include <cstdint>

#define TIME_CAN_ID 0x04

// Global variables
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_volt_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_curr_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_temp_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_bms_temp_pub;
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr battery_level_pub;
rclcpp::Publisher<hightorque_power::msg::PowerSwitch>::SharedPtr power_switch_pub;
rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr bms_error_pub;
rclcpp::Publisher<hightorque_power::msg::VersionInfo>::SharedPtr version_pub;
rclcpp::Subscription<hightorque_power::msg::PowerSwitch>::SharedPtr power_switch_sub;
livelybot_can::CAN_Driver can_handler("can0");

typedef enum __attribute__((packed))
{
    VERSION_Ctrl = 0,  // Control algorithm
    VERSION_SDK = 1,   // SDK
    VERSION_Oled = 2,  // OLED
    VERSION_Power = 3, // Power board
    VERSION_Com = 4,   // Communication board
    VERSION_BMS = 5,   // Battery
    VERSION_Mot_L = 6, // Motor minimum version
    VERSION_Mot_U = 7, // Motor maximum version
} version_type_s;

uint8_t calculated_crc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];
    }
    return crc;
}

uint8_t parse_version(char *version, uint8_t *data, uint8_t len)
{
    if (len < 8)
    {
        return 0;
    }
    if (calculated_crc(data, 7) != data[7])
    {
        return 0;
    }
    snprintf(version, 9, "%d.%d.%d", data[2], data[3], data[4]);
    return 1;
}

// Custom CAN receive callback function
void custom_can_recv_parse(int can_id, unsigned char *data, unsigned char len)
{
// BMS address definition
#define BMS_ADDR 0x06
// Power board address definition
#define POWER_SWITCH_ADDR 0x07

    uint8_t dev_addr;
    uint8_t data_type;
    uint8_t append_flag;

    dev_addr = can_id >> 7;
    data_type = (can_id >> 1) & 0x3F;
    append_flag = can_id & 0x01;

    if (!append_flag && dev_addr == BMS_ADDR && data_type == 0x01)
    {
        // Publish battery voltage
        auto volt_msg = std_msgs::msg::Float32();
        volt_msg.data = (*(int16_t *)&data[0]) / 100.0f;
        battery_volt_pub->publish(volt_msg);

        // Publish battery current
        auto curr_msg = std_msgs::msg::Float32();
        curr_msg.data = (*(int16_t *)&data[2]) / 100.0f;
        battery_curr_pub->publish(curr_msg);

        // Publish battery temperature
        auto temp_msg = std_msgs::msg::Float32();
        temp_msg.data = (float)(int8_t)data[5];
        battery_temp_pub->publish(temp_msg);

        // Publish battery board temperature
        auto temp_bms_msg = std_msgs::msg::Float32();
        temp_bms_msg.data = (float)(int8_t)data[4];
        battery_bms_temp_pub->publish(temp_bms_msg);

        auto level_msg = std_msgs::msg::UInt8();
        level_msg.data = data[6];
        battery_level_pub->publish(level_msg);
    }
    else if (!append_flag && dev_addr == BMS_ADDR && data_type == 0x02)
    {
        // Publish BMS error information
        auto error_msg = std_msgs::msg::Int8();
        error_msg.data = data[0];
        bms_error_pub->publish(error_msg);
    }
    else if (!append_flag && dev_addr == POWER_SWITCH_ADDR && data_type == 0x02)
    {
        // Handle power board switch status
        auto power_switch_msg = hightorque_power::msg::PowerSwitch();
        power_switch_msg.control_switch = data[0];
        power_switch_msg.power_switch = data[1];
        power_switch_pub->publish(power_switch_msg);
    }
    else if (!append_flag && dev_addr == POWER_SWITCH_ADDR && data_type == 0x05)
    {
        char buff[9];
        if (parse_version(buff, data, len))
        {
            auto msg = hightorque_power::msg::VersionInfo();
            msg.device_type = VERSION_Power;
            msg.version = std::string(buff);
            version_pub->publish(msg);
        }
    }
    else if (!append_flag && dev_addr == BMS_ADDR && data_type == 0x05)
    {
        char buff[9];
        if (parse_version(buff, data, len))
        {
            auto msg = hightorque_power::msg::VersionInfo();
            msg.device_type = VERSION_BMS;
            msg.version = std::string(buff);
            version_pub->publish(msg);
        }
    }
}

void power_switch_callback(const hightorque_power::msg::PowerSwitch::SharedPtr msg)
{
#define ORANGEPI_ADDR 0x01
    uint32_t can_id = (ORANGEPI_ADDR << 7) | (1 << 1) | 0;
    uint8_t data[2];

    data[0] = msg->control_switch;
    data[1] = msg->power_switch;

    can_handler.send(can_id, data, 2);
}

void com_power_control_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    unsigned char data[1];
    data[0] = msg->data;
    can_handler.send(0x084, data, 1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::cout << "节点启动" << std::endl;

    auto node = std::make_shared<rclcpp::Node>("power_node");

    // Initialize publishers
    battery_volt_pub = node->create_publisher<std_msgs::msg::Float32>("battery_voltage", 1);
    battery_curr_pub = node->create_publisher<std_msgs::msg::Float32>("battery_current", 1);
    battery_temp_pub = node->create_publisher<std_msgs::msg::Float32>("battery_temperature", 1);
    battery_bms_temp_pub = node->create_publisher<std_msgs::msg::Float32>("battery_bms_temperature", 1);
    battery_level_pub = node->create_publisher<std_msgs::msg::UInt8>("battery_level", 1);
    power_switch_pub = node->create_publisher<hightorque_power::msg::PowerSwitch>("power_switch_state", 1);
    bms_error_pub = node->create_publisher<std_msgs::msg::Int8>("bms_error", 1);
    version_pub = node->create_publisher<hightorque_power::msg::VersionInfo>("version", 10);

    can_handler.start_callback(custom_can_recv_parse);

    // Initialize subscribers
    power_switch_sub = node->create_subscription<hightorque_power::msg::PowerSwitch>(
        "power_switch_control", 1, power_switch_callback);

    auto com_power_control = node->create_subscription<std_msgs::msg::UInt8>(
        "com_power_control", 1, com_power_control_callback);

    uint32_t can_id = (TIME_CAN_ID << 7) | (1 << 1) | 0;
    std::time_t last_sent_ts = 0;

    // Main loop
    rclcpp::Rate r(10);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        // Get current UTC timestamp (seconds)
        std::time_t now = std::time(nullptr);

        // Update timestamp and send once per second
        if (now != last_sent_ts)
        {
            last_sent_ts = now;

            // Send Beijing time
            uint32_t unix_ts = static_cast<uint32_t>(now);

            uint8_t data[4];
            data[0] = (unix_ts >> 0) & 0xFF;
            data[1] = (unix_ts >> 8) & 0xFF;
            data[2] = (unix_ts >> 16) & 0xFF;
            data[3] = (unix_ts >> 24) & 0xFF;

            can_handler.send(can_id, data, 4);
        }
        r.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
