#include "motor/motor_base.h"

namespace hightorque
{

    MotorState HtdwMotor::getMotorBackState() const
    {
        static auto lastWarnTime = std::chrono::system_clock::now();
        auto now = std::chrono::system_clock::now();

        auto rawData = *rbPtr_->Motors[index_]->get_current_motor_state();

        MotorState motorBackData;
        motorBackData.time = rawData.time;
        motorBackData.id = index_;
        motorBackData.mode = rawData.mode;
        motorBackData.fault = rawData.fault;

        // 获取状态时：先乘 direction，再减 offset
        motorBackData.position = rawData.position * direction_ - urdf_offset_;
        motorBackData.velocity = rawData.velocity * direction_;
        motorBackData.torque = rawData.torque * direction_;

        motorBackData.num = rawData.num;
        motorBackData.name = name_;

        if (motorBackData.fault > 0)
        {
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - lastWarnTime).count();
            if (duration >= 2)
            {
                std::cerr << "[" << index_ << "]" << name_ << " fault code: " << static_cast<unsigned long>(motorBackData.fault) << std::endl;
                lastWarnTime = now;
            }
        }
        return motorBackData;
    }

    void HtdwMotor::setMotor(float position, float velocity, float torque, float kp, float ki, float kd, float acc, float voltage, float current, MotorControlType type)
    {
        // 写入时：先加 offset，再乘 direction
        float hwPosition = (position + urdf_offset_) * direction_;
        float hwVelocity = velocity * direction_;
        float hwTorque = torque * direction_;

        // TODO:修改为传入参数
        // 执行预设动作 限位大一点
        auto highest = type == POS_VEL_MAX_TQE ? 6.0 : highest_;
        auto lowest = type == POS_VEL_MAX_TQE ? -6.0 : lowest_;
        if (hwPosition > highest)
        {
            std::cerr << "[" << index_ << "]" << name_ << ":" << hwPosition << " > " << highest << std::endl;
            hwPosition = highest;
        }
        if (hwPosition < lowest)
        {
            std::cerr << "[" << index_ << "]" << name_ << ":" << hwPosition << " < " << lowest << std::endl;
            hwPosition = lowest;
        }
        lastCmd_ = MotorControlCmd(position, velocity, torque,
                                   kp, ki, kd,
                                   acc, voltage, current, type);
        switch (type)
        {
            case POS:
                rbPtr_->Motors[index_]->position(hwPosition);
                break;
            case VEL:
                rbPtr_->Motors[index_]->velocity(hwVelocity);
                break;
            case TQE:
                rbPtr_->Motors[index_]->torque(hwTorque);
                break;
            case VOL:
                rbPtr_->Motors[index_]->voltage(voltage);
                break;
            case CUR:
                rbPtr_->Motors[index_]->current(current);
                break;
            case POS_VEL_MAX_TQE:
                rbPtr_->Motors[index_]->pos_vel_MAXtqe(hwPosition, hwVelocity, hwTorque);
                break;
            case POS_VEL_TQE_RKP_RKD:
                rbPtr_->Motors[index_]->pos_vel_tqe_kp_kd(hwPosition, hwVelocity, hwTorque, kp, kd);
                break;
            case POS_VEL_RKP_RKD:
                rbPtr_->Motors[index_]->pos_vel_kp_kd(hwPosition, hwVelocity, kp, kd);
                break;
            case POS_VEL_TQE_KP_KD:
                rbPtr_->Motors[index_]->pos_vel_tqe_kp_kd(hwPosition, hwVelocity, hwTorque, kp, kd);
                break;
            case POS_VEL_KP_KD:
                rbPtr_->Motors[index_]->pos_vel_kp_kd(hwPosition, hwVelocity, kp, kd);
                break;
            case POS_VEL_ACC:
                rbPtr_->Motors[index_]->pos_vel_acc(hwPosition, hwVelocity, acc);
                break;
            case POS_VEL_TQE_KP_KD2:
                // rbPtr_->Motors[index_]->pos_vel_tqe_kp_kd2(hw_position, hw_velocity, hw_torque, kp, kd);
                break;
        }
    }

    void HtdwMotor::setMotor(MotorControlCmd cmd)
    {
        // 写入时：先加 offset，再乘 direction
        float hwPosition = (cmd.position + urdf_offset_) * direction_;
        float hwVelocity = cmd.velocity * direction_;
        float hwTorque = cmd.torque;

        // TODO:修改为传入参数
        // 执行预设动作 限位大一点
        auto highest = cmd.type == POS_VEL_MAX_TQE ? 6.0 : highest_;
        auto lowest = cmd.type == POS_VEL_MAX_TQE ? -6.0 : lowest_;
        if (hwPosition > highest)
        {
            std::cerr << "[" << index_ << "]" << name_ << ":" << hwPosition << " > " << highest << std::endl;
            hwPosition = highest;
        }
        if (hwPosition < lowest)
        {
            std::cerr << "[" << index_ << "]" << name_ << ":" << hwPosition << " < " << lowest << std::endl;
            hwPosition = lowest;
        }
        float kp = cmd.kp, ki = cmd.ki, kd = cmd.kd,
              acc = cmd.acc, voltage = cmd.voltage, current = cmd.current;

        lastCmd_ = MotorControlCmd(cmd.position, cmd.velocity, cmd.torque,
                                   kp, ki, kd,
                                   acc, voltage, current, cmd.type);
        switch (cmd.type)
        {

            case POS:
                rbPtr_->Motors[index_]->position(hwPosition);
                break;
            case VEL:
                rbPtr_->Motors[index_]->velocity(hwVelocity);
                break;
            case TQE:
                rbPtr_->Motors[index_]->torque(hwTorque);
                break;
            case VOL:
                rbPtr_->Motors[index_]->voltage(voltage);
                break;
            case CUR:
                rbPtr_->Motors[index_]->current(current);
                break;
            case POS_VEL_MAX_TQE:
                rbPtr_->Motors[index_]->pos_vel_MAXtqe(hwPosition, hwVelocity, hwTorque);
                break;
            case POS_VEL_TQE_RKP_RKD:
                rbPtr_->Motors[index_]->pos_vel_tqe_kp_kd(hwPosition, hwVelocity, hwTorque, kp, kd);
                break;
            case POS_VEL_RKP_RKD:
                rbPtr_->Motors[index_]->pos_vel_kp_kd(hwPosition, hwVelocity, kp, kd);
                break;
            case POS_VEL_TQE_KP_KD:
                rbPtr_->Motors[index_]->pos_vel_tqe_kp_kd(hwPosition, hwVelocity, hwTorque, kp, kd);
                break;
            case POS_VEL_KP_KD:
                rbPtr_->Motors[index_]->pos_vel_kp_kd(hwPosition, hwVelocity, kp, kd);
                break;
            case POS_VEL_ACC:
                rbPtr_->Motors[index_]->pos_vel_acc(hwPosition, hwVelocity, acc);
                break;
            case POS_VEL_TQE_KP_KD2:
                // rbPtr_->Motors[index_]->pos_vel_tqe_kp_kd2(hwPosition, hwVelocity, hwTorque, kp, kd);
                break;
        }
    }

    void HtdwMotor::setMotorRelative(float relativePosition, float velocity, float torque, float kp, float ki, float kd, float acc, float voltage, float current, MotorControlType type)
    {
        // 获取当前位置
        float currentPosition = getMotorBackState().position;

        // 计算目标位置 = 当前位置 + 相对位移
        float position = currentPosition + relativePosition;

        setMotor(position, velocity, torque,
                 kp, ki, kd,
                 acc, voltage, current, type);
    }

    void HtdwMotor::setMotorRelative(MotorControlCmd cmd)
    {
        // 获取当前位置
        float currentPosition = getMotorBackState().position;

        // 计算目标位置 = 当前位置 + 相对位移
        cmd.position += currentPosition;

        setMotor(cmd);
    }

    void HtdwMotor::protectMotor()
    {
        float kp = 0.0, kd = 1.0;
        setMotor(0, 0, 0,
                 kp, 0, kd,
                 0, 0, 0);
        lastCmd_ = MotorControlCmd(0, 0, 0,
                                   kp, 0, kd,
                                   0, 0, 0);
    }

    void HtdwMotor::keepMotorLastCmd()
    {
        setMotor(lastCmd_.position, lastCmd_.velocity, lastCmd_.torque,
                 lastCmd_.kp, lastCmd_.ki, lastCmd_.kd,
                 lastCmd_.acc, lastCmd_.voltage, lastCmd_.current, lastCmd_.type);
    }
}
