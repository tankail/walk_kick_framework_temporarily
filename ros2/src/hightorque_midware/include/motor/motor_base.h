#ifndef MOTOR_BASE_H_
#define MOTOR_BASE_H_

#include "hardware/robot.hpp"
#include "motor/motor_types.h"
#include "iostream"
#include <string>
namespace hightorque
{
    enum MotorControlType
    {
        POS = 1,                 // position
        VEL = 2,                 // velocity
        TQE = 3,                 // torque
        VOL = 4,                 // voltage
        CUR = 5,                 // current
        POS_VEL_MAX_TQE = 6,     // position velocity max_tqe
        POS_VEL_TQE_RKP_RKD = 7, // poseition velocity torque rkp rkd
        POS_VEL_RKP_RKD = 8,     //  position velocity rkp rkd
        POS_VEL_TQE_KP_KD = 9,   // position velocity torque kp kd
        POS_VEL_KP_KD = 10,      // position velocity kp kd
        POS_VEL_ACC = 11,        // position velocity acceleration
        POS_VEL_TQE_KP_KD2 = 12, // position velocity torque kp kd
    };
    class MotorControlCmd
    {
        public:
            MotorControlCmd(
                float position = 0.0f,                                      // 目标位置（默认0）
                float velocity = 0.0f,                                      // 目标速度（默认0）
                float torque = 0.0f,                                        // 目标扭矩（默认0）
                float kp = 0.0f,                                            // 位置比例增益（默认0）
                float ki = 0.0f,                                            // 位置积分增益（默认0）
                float kd = 0.0f,                                            // 位置微分增益（默认0）
                float acc = 0.0f,                                           // 加速度限制（默认0）
                float voltage = 0.0f,                                       // 电压限制（默认0）
                float current = 0.0f,                                       // 电流限制（默认0）
                MotorControlType type = MotorControlType::POS_VEL_TQE_KP_KD // 电机控制类型 （默认位置 速度 扭矩 kp kd）
                ) : position(position), velocity(velocity), torque(torque),
                    kp(kp), ki(ki), kd(kd),
                    acc(acc), voltage(voltage), current(current), type(type)
            {
            }

            float position, velocity, torque, kp, ki, kd, acc, voltage, current;
            MotorControlType type;

            std::string toString()
            {
                std::ostringstream oss;
                oss << "MotorControlCmd (\n"
                    << " (" << position << ", " << velocity << ", " << torque << ")\n"
                    << " (" << kp << ", " << ki << ", " << kd << ")\n"
                    << " (" << acc << ", " << voltage << ", " << current << ")\n"
                    << " (" << type << ")\n";
                return oss.str();
            }
    };

    /** =========================================================================
     * @brief 电机控制基类接口 MotorBase
     *
     * 该抽象类定义了电机通用控制接口，包括设置绝对/相对目标、获取状态、
     * 以及保持上一次控制指令等方法。所有电机驱动实现需继承并实现该接口。
     * ========================================================================= */
    class MotorBase
    {
        public:
            virtual ~MotorBase() {};
            /**
             * @brief 获取电机名称
             * @return std::string 电机名称
             */
            virtual std::string getName() const = 0;
            /**
             * @brief 获取电机索引（通常用于映射表或数组）
             * @return int 电机在系统中的唯一编号
             */
            virtual int getIndex() const = 0;
            /**
             * @brief 获取电机反馈状态
             * @return MotorState 当前电机反馈状态结构体
             */
            virtual MotorState getMotorBackState() const = 0;
            /**
             * @brief 设置电机控制命令（绝对位置）
             *
             * @param position 目标位置
             * @param velocity 目标速度
             * @param torque   期望力矩
             * @param kp       位置环增益
             * @param ki       积分增益
             * @param kd       微分增益
             * @param acc      加速度限制
             * @param voltage  电压限制
             * @param current  电流限制
             * @param type     电机控制类型
             */
            virtual void setMotor(float position, float velocity, float torque, float kp, float ki, float kd, float acc, float voltage, float current, MotorControlType type = MotorControlType::POS_VEL_TQE_KP_KD) = 0;
            /**
             * @brief 设置电机控制命令（使用结构体封装）
             * @param cmd MotorControlCmd 控制命令结构体
             */
            virtual void setMotor(MotorControlCmd cmd) = 0;
            /**
             * @brief 设置电机相对移动命令（相对于当前位置）
             */
            virtual void setMotorRelative(float position, float velocity, float torque, float kp, float ki, float kd, float acc, float voltage, float current, MotorControlType type = MotorControlType::POS_VEL_TQE_KP_KD) = 0;
            /**
             * @brief 设置电机相对移动命令（结构体封装）
             */
            virtual void setMotorRelative(MotorControlCmd cmd) = 0;
            /**
             * @brief 保持上一次电机状态输出（会比较“硬”但可动）
             */
            virtual void protectMotor() = 0;
            /**
             * @brief 保持上一次控制命令,让电机保持不动
             */
            virtual void keepMotorLastCmd() = 0;

            MotorControlCmd lastCmd_;
    };
    /** ======================== 电机控制基类 MotorBase 定义结束 ======================== */

    /** =========================================================================
     * @brief 电机驱动实现类：HtdwMotor
     *
     * 基于 MotorBase 接口的实际实现，使用 LivelyBot 串口驱动实现具体HTDW-5047-36-NE电机控制。
     * 支持绝对/相对控制、状态反馈、命令保持等功能。
     * ========================================================================= */
    class HtdwMotor : public MotorBase
    {
        public:
            HtdwMotor() = delete;
            /**
             * @brief 构造函数
             *
             * @param rbPtr 共享的 LivelyBot 串口机器人实例
             * @param index 电机编号索引（全局唯一）
             * @param name  电机名称（如 "r_ankle_roll_joint"）
             * @param lowest 电机限位
             * @param highest 电机限位
             * @param direction 电机方向（1.0 或 -1.0）
             * @param urdf_offset URDF偏移量
             */
            HtdwMotor(std::shared_ptr<hightorque_robot::robot> rbPtr, int index, const std::string& name,
                      double lowest, double highest, double direction = 1.0, double urdf_offset = 0.0)
                : rbPtr_(rbPtr), index_(index), name_(name), lowest_(lowest), highest_(highest),
                  direction_(direction), urdf_offset_(urdf_offset) {};

            /**
             * @brief 获取电机索引
             * @return int 当前电机索引
             */
            inline int getIndex() const override
            {
                return index_;
            }

            /**
             * @brief 获取电机名称
             * @return std::string 电机名称
             */
            inline std::string getName() const override
            {
                return name_;
            }

            /**
             * @brief 获取电机反馈状态（位置、速度、电流等）
             * @return MotorState 当前反馈状态
             */
            MotorState getMotorBackState() const override;

            /**
             * @brief 设置电机控制命令（绝对位置）
             */
            void setMotor(float position, float velocity, float torque, float kp, float ki, float kd, float acc, float voltage, float current, MotorControlType type = MotorControlType::POS_VEL_TQE_KP_KD) override;

            /**
             * @brief 设置电机控制命令（封装结构体形式）
             */
            void setMotor(MotorControlCmd cmd) override;

            /**
             * @brief 设置电机相对控制命令（相对于当前位置）
             */
            void setMotorRelative(float position, float velocity, float torque, float kp, float ki, float kd, float acc, float voltage, float current, MotorControlType type = MotorControlType::POS_VEL_TQE_KP_KD) override;
            /**
             * @brief 设置电机相对控制命令（封装结构体形式）
             */
            void setMotorRelative(MotorControlCmd cmd) override;
            /**
             * @brief 保持上一次电机状态输出（会比较“硬”但可动）
             */
            void protectMotor() override;
            /**
             * @brief 保持上一次控制命令,让电机保持不动
             */
            void keepMotorLastCmd() override;

        private:
            std::shared_ptr<hightorque_robot::robot> rbPtr_; // 串口机器人对象指针
            MotorState motorState_;                          // 当前电机反馈状态
            int index_;                                      // 电机索引
            double lowest_, highest_;                        // 保护
            std::string name_;                               // 电机名称
            double direction_;                               // 电机方向（1.0 或 -1.0）
            double urdf_offset_;                             // URDF偏移量
    };
    /** ======================== HtdwMotor 实现类定义结束 ======================== */

}

#endif
