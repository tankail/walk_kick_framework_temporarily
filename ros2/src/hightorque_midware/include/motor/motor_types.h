#ifndef MOTOR_TYPES_H_
#define MOTOR_TYPES_H_

#include <cstdint>
#include <string>

namespace hightorque
{

    /**
     * @brief 电机反馈状态结构
     */
    struct MotorState
    {
            double time;      // 时间戳
            std::string name; // 电机名称
            uint8_t id;       // 电机ID
            uint8_t mode;     // 模式
            uint8_t fault;    // 故障状态
            float position;   // 位置（rad）
            float velocity;   // 速度（rad/s）
            float torque;     // 力矩（Nm）
            int num;          // 编号

            MotorState()
                : time(0.0), id(0), mode(0), fault(0),
                  position(0.0f), velocity(0.0f), torque(0.0f), num(0) {}

            std::string toString() const
            {
                return "MotorState{name='" + name + "', id=" + std::to_string(id) +
                       ", mode=" + std::to_string(mode) +
                       ", fault=" + std::to_string(fault) +
                       ", position=" + std::to_string(position) +
                       ", velocity=" + std::to_string(velocity) +
                       ", torque=" + std::to_string(torque) +
                       ", num=" + std::to_string(num) + "}";
            }
    };

} // namespace hightorque

#endif // MOTOR_TYPES_H_
