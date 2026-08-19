#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <yesense_imu/yesense_driver.h>
#include <memory>

class YesenseImuNode : public rclcpp::Node
{
public:
    YesenseImuNode() : Node("yesense_imu_node")
    {
        // 声明参数
        this->declare_parameter("publish_rate", 100.0);   // 默认100Hz
        this->declare_parameter("config_file", "");       // IMU配置文件路径

        // 获取参数
        double publish_rate = this->get_parameter("publish_rate").as_double();
        std::string config_file = this->get_parameter("config_file").as_string();

        // 初始化 IMU 驱动
        if (config_file.empty()) {
            RCLCPP_INFO(this->get_logger(), "使用默认配置文件路径");
            yesense_driver_ = std::make_shared<yesense::YesenseDriver>();
        } else {
            RCLCPP_INFO(this->get_logger(), "使用指定配置文件: %s", config_file.c_str());
            yesense_driver_ = std::make_shared<yesense::YesenseDriver>(config_file);
        }

        // 创建发布者
        rpy_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "rpy", 10);
        acc_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "acc", 10);
        gyro_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "gyro", 10);
        quat_pub_ = this->create_publisher<geometry_msgs::msg::Quaternion>(
            "quat", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "imu", 10);

        // 创建定时器
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&YesenseImuNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Yesense IMU 节点已启动");
        RCLCPP_INFO(this->get_logger(), "发布频率: %.1f Hz", publish_rate);
        RCLCPP_INFO(this->get_logger(), "话题列表:");
        RCLCPP_INFO(this->get_logger(), "  - rpy   (姿态角)");
        RCLCPP_INFO(this->get_logger(), "  - acc   (加速度)");
        RCLCPP_INFO(this->get_logger(), "  - gyro  (角速度)");
        RCLCPP_INFO(this->get_logger(), "  - quat  (四元数)");
        RCLCPP_INFO(this->get_logger(), "  - imu   (标准IMU消息)");
    }

    ~YesenseImuNode()
    {
        RCLCPP_INFO(this->get_logger(), "Yesense IMU 节点已关闭");
    }

private:
    void timer_callback()
    {
        double rpy[3], acc[3], gyro[3], quat[4];

        // 从驱动获取数据
        yesense_driver_->get_rpy(rpy);
        yesense_driver_->get_accel(acc);
        yesense_driver_->get_gyro(gyro);
        yesense_driver_->get_quat(quat);

        // 获取当前时间戳
        auto stamp = this->now();

        // 发布姿态角 (RPY)
        geometry_msgs::msg::Vector3 rpy_msg;
        rpy_msg.x = rpy[0];  // Roll
        rpy_msg.y = rpy[1];  // Pitch
        rpy_msg.z = rpy[2];  // Yaw
        rpy_pub_->publish(rpy_msg);

        // 发布加速度
        geometry_msgs::msg::Vector3 acc_msg;
        acc_msg.x = acc[0];
        acc_msg.y = acc[1];
        acc_msg.z = acc[2];
        acc_pub_->publish(acc_msg);

        // 发布角速度
        geometry_msgs::msg::Vector3 gyro_msg;
        gyro_msg.x = gyro[0];
        gyro_msg.y = gyro[1];
        gyro_msg.z = gyro[2];
        gyro_pub_->publish(gyro_msg);

        // 发布四元数
        geometry_msgs::msg::Quaternion quat_msg;
        quat_msg.w = quat[0];
        quat_msg.x = quat[1];
        quat_msg.y = quat[2];
        quat_msg.z = quat[3];
        quat_pub_->publish(quat_msg);

        // 发布标准 IMU 消息
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = "imu_link";

        // 设置四元数
        imu_msg.orientation.w = quat[0];
        imu_msg.orientation.x = quat[1];
        imu_msg.orientation.y = quat[2];
        imu_msg.orientation.z = quat[3];

        // 设置角速度 (需要转换为 rad/s)
        imu_msg.angular_velocity.x = gyro[0] * M_PI / 180.0;
        imu_msg.angular_velocity.y = gyro[1] * M_PI / 180.0;
        imu_msg.angular_velocity.z = gyro[2] * M_PI / 180.0;

        // 设置线性加速度
        imu_msg.linear_acceleration.x = acc[0];
        imu_msg.linear_acceleration.y = acc[1];
        imu_msg.linear_acceleration.z = acc[2];

        imu_pub_->publish(imu_msg);
    }

    // 成员变量
    std::shared_ptr<yesense::YesenseDriver> yesense_driver_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rpy_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr acc_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gyro_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr quat_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<YesenseImuNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "异常: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
