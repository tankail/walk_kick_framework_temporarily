#ifndef COMMON_MOTOR_GROUP_H_
#define COMMON_MOTOR_GROUP_H_
#include "motor/robot_motor_group.h"

namespace hightorque
{
    /** =========================================================================
     * @brief CommonMotorGroup 机器人整体电机组控制类
     *
     * 该类负责管理多个命名的电机组（MotorGroup），用于按名称访问和控制子模块（如腿、手臂等）。
     * 提供对电机组状态获取、指令发送、保持状态等接口。
     * ========================================================================= */
    class CommonMotorGroup : public RobotMotorGroup
    {
        public:
            CommonMotorGroup() = delete;

            /**
             * @brief 构造函数
             *
             * @param rbPtr Livelybot 机器人底层指针
             * @param mapIndex 电机映射索引列表（与底层通讯映射用）
             * @param names 每组电机组的名称，用于标识电机（如 "r_ankle_roll_joint"）
             * @param lowest 电机下限位
             * @param highest 电机上限位
             * @param direction 电机方向（1.0 或 -1.0）
             * @param offset URDF偏移量
             */
            CommonMotorGroup(std::shared_ptr<hightorque_robot::robot> rbPtr,
                             std::vector<int> mapIndex,
                             std::vector<std::string> names,
                             std::vector<double> lowest,
                             std::vector<double> highest,
                             std::vector<double> direction,
                             std::vector<double> offset);
    };
}

#endif
