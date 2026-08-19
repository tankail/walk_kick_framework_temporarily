#ifndef MOTOR_GROUP_H_
#define MOTOR_GROUP_H_
#include "motor.hpp"
#include "motor/motor_base.h"

namespace hightorque
{
    /** =========================================================================
     * @brief MotorGroup 电机组控制类
     *
     * 管理一组继承自 MotorBase 的电机对象，统一控制多个电机的状态、指令发送、
     * 相对运动等操作。
     * ========================================================================= */
    class MotorGroup
    {
        public:
            /**
             * @brief 使用 MotorBase 指针列表构造电机组
             *
             * @param size 电机数量（用于索引或安全判断）
             * @param motors MotorBase 派生类实例列表
             */
            MotorGroup(int size, std::vector<std::shared_ptr<MotorBase>> motors)
                : size_(size), motors_(std::move(motors))
            {
                motorsMap_.reserve(motors_.size());
                for (const auto& motor : motors_)
                {
                    index_.push_back(motor->getIndex());
                    motorsMap_.emplace(motor->getName(), motor);
                }
            }

            /**
             * @brief 模板构造函数，支持从任意 MotorBase 派生类构造
             *
             * 自动将 std::shared_ptr<Derived> 向上转型为 shared_ptr<MotorBase>
             *
             * @tparam Derived MotorBase 的派生类
             * @param size 电机数量
             * @param derivedMotors 派生类电机对象列表
             */
            template <typename Derived,
                      typename = std::enable_if_t<std::is_base_of_v<MotorBase, Derived>>>
            MotorGroup(int size, const std::vector<std::shared_ptr<Derived>>& derivedMotors)
                : size_(size)
            {
                motors_.reserve(derivedMotors.size());
                motorsMap_.reserve(derivedMotors.size());

                for (const auto& motor : derivedMotors)
                {
                    index_.push_back(motor->getIndex());
                    motors_.push_back(motor); // 向上转型
                    motorsMap_.emplace(motor->getName(), motor);
                }
            }

            virtual ~MotorGroup()
            {
                motorsMap_.clear();
                motors_.clear();
            }

            /**
             * @brief 获取电机组大小
             * @return int 电机数量
             */
            inline int size()
            {
                return size_;
            }

            inline const std::vector<int>& getIndex()
            {
                return index_;
            }

            /**
             * @brief 获取所有电机的反馈状态
             * @return std::vector<MotorState> 状态数组
             */
            std::vector<MotorState> getMotorsBackState() const;

            /**
             * @brief 获取所有电机按index升序的反馈状态
             * @return std::vector<MotorState> 状态数组
             */
            std::vector<MotorState> getMotorsBackStateByIndexUp() const;

            /**
             * @brief 获取所有电机状态的字符串表示（调试用）
             * @return std::string 状态描述字符串
             */
            std::string getMotorStateString() const;

            /**
             * @brief 向所有电机发送绝对控制命令
             * @param cmd std::vector<MotorControlCmd> 控制命令向量
             */
            void setMotors(const std::vector<MotorControlCmd>& cmd) const;

            /**
             * @brief 向所有电机发送相对控制命令
             * @param cmd std::vector<MotorControlCmd> 控制命令向量
             */
            void setMotorsRelative(const std::vector<MotorControlCmd>& cmd) const;

            /**
             * @brief 向所有电机按index升序发送绝对控制命令
             * @param cmd std::vector<MotorControlCmd> 控制命令向量
             */
            void setMotorsByIndexUp(const std::vector<MotorControlCmd>& cmd) const;

            /**
             * @brief 向所有电机按index升序发送相对控制命令
             * @param cmd std::vector<MotorControlCmd> 控制命令向量
             */
            void setMotorsRelativeByIndexUp(const std::vector<MotorControlCmd>& cmd) const;

            /**
             * @brief 所有电机保持当前状态,（会比较“硬”但可动）
             */
            void protectMotor() const;

            /**
             * @brief 所有电机保持上一次命令,让电机保持不动
             */
            virtual void keepMotorsLastCmd() const
            {
                for (const auto& motor : motors_)
                {
                    motor->keepMotorLastCmd();
                }
            }

            /**
             * @brief 获取所有电机对象的共享指针列表
             * @return 包含所有 MotorBase 派生对象的 std::vector
             */
            virtual std::vector<std::shared_ptr<MotorBase>> getMotors() const
            {
                return motors_;
            }

            /**
             * @brief 获取指定电机对象的共享指针列表
             * @return MotorBase 派生对象的共享指针
             */
            virtual std::shared_ptr<MotorBase> getMotor(const std::string& name) const;

        private:
            std::vector<std::shared_ptr<MotorBase>> motors_;                        // 电机指针数组
            std::unordered_map<std::string, std::shared_ptr<MotorBase>> motorsMap_; // key:电机名称 电机指针数组
            int size_;                                                              // 电机数量
            std::vector<int> index_;                                                // 电机编号
    };

}

#endif
