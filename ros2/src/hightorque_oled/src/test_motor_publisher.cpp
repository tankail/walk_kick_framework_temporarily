#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>

using namespace std::chrono_literals;

class MotorStatePublisher : public rclcpp::Node
{
public:
    MotorStatePublisher() : Node("motor_state_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/error_joint_states", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&MotorStatePublisher::publish_motor_states, this));

        RCLCPP_INFO(this->get_logger(), "Motor state publisher started, publishing 23 motors");
    }

private:
    void publish_motor_states()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();

        // 设置23个电机
        const int MOTOR_COUNT = 23;
        msg.name.resize(MOTOR_COUNT);
        msg.position.resize(MOTOR_COUNT);
        msg.velocity.resize(MOTOR_COUNT);
        msg.effort.resize(MOTOR_COUNT);

        for(int i = 0; i < MOTOR_COUNT; i++)
        {
            // 设置电机名称
            msg.name[i] = "motor_" + std::to_string(i + 1);

            // 模拟电机状态:
            // 大部分电机正常 (position > -900.0)
            // 少数电机模拟故障 (position < -900.0)
            if(i == 5 || i == 12 || i == 18)  // 模拟第6、13、19号电机故障
            {
                msg.position[i] = -1000.0;  // 故障状态
                msg.velocity[i] = 0.0;
                msg.effort[i] = 0.0;
            }
            else
            {
                // 正常状态，设置一些模拟的位置值
                msg.position[i] = static_cast<double>(i) * 0.1;  // 0.0, 0.1, 0.2, ...
                msg.velocity[i] = 0.5;
                msg.effort[i] = 1.0;
            }
        }

        publisher_->publish(msg);

        // 每秒打印一次状态
        static int count = 0;
        if(++count >= 10)
        {
            count = 0;
            RCLCPP_INFO(this->get_logger(), "Published motor states: 23 motors (motors 6, 13, 19 in error state)");
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
