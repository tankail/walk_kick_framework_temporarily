#include "motor/motor_group.h"
#include <algorithm>

namespace hightorque
{
    std::vector<MotorState> MotorGroup::getMotorsBackState() const
    {
        std::vector<MotorState> motorsBackState;
        for (const auto& motor : motors_)
        {
            motorsBackState.push_back(motor->getMotorBackState());
        }
        return motorsBackState;
    }

    std::vector<MotorState> MotorGroup::getMotorsBackStateByIndexUp() const
    {
        // 创建按索引排序的电机副本
        std::vector<std::shared_ptr<MotorBase>> sortedMotors = motors_;
        std::sort(sortedMotors.begin(), sortedMotors.end(),
                  [](const auto& a, const auto& b) {
                      return a->getIndex() < b->getIndex();
                  });

        std::vector<MotorState> motorsBackState;
        for (const auto& motor : sortedMotors)
        {
            motorsBackState.push_back(motor->getMotorBackState());
        }
        return motorsBackState;
    }

    std::string MotorGroup::getMotorStateString() const
    {
        std::stringstream ss;
        ss << "MotorGroup State (size=" << size_ << "):\n";

        for (int i = 0; i < size_; i++)
        {
            const auto& motor = motors_[i];
            auto backState = motor->getMotorBackState();

            ss << "  [" << i << "] " << motor->getName()
               << " (index=" << motor->getIndex() << "):\n"
               << "    Position: " << backState.position << "\n"
               << "    Velocity: " << backState.velocity << "\n"
               << "    Torque:   " << backState.torque << "\n";
        }

        return ss.str();
    }

    void MotorGroup::setMotors(const std::vector<MotorControlCmd>& cmd) const
    {
        if (cmd.size() < static_cast<size_t>(size_))
        {
            std::cerr << "MotorGroup cmd size " << cmd.size() << " error, expected at least " << size_ << std::endl;
            return;
        }
        for (int i = 0; i < size_; i++)
        {
            const auto motor = motors_[i];
            motor->setMotor(cmd[i]);
        }
    }

    void MotorGroup::setMotorsRelative(const std::vector<MotorControlCmd>& cmd) const
    {
        if (cmd.size() < static_cast<size_t>(size_))
        {
            std::cerr << "MotorGroup cmd size " << cmd.size() << " error, expected at least " << size_ << std::endl;
            return;
        }
        for (int i = 0; i < size_; i++)
        {
            const auto motor = motors_[i];
            motor->setMotorRelative(cmd[i]);
        }
    }

    void MotorGroup::setMotorsByIndexUp(const std::vector<MotorControlCmd>& cmd) const
    {
        // 创建按索引排序的电机副本
        std::vector<std::shared_ptr<MotorBase>> sortedMotors = motors_;
        std::sort(sortedMotors.begin(), sortedMotors.end(),
                  [](const auto& a, const auto& b) {
                      return a->getIndex() < b->getIndex();
                  });

        if (cmd.size() < sortedMotors.size())
        {
            std::cerr << "MotorGroup cmd size " << cmd.size() << " error, expected at least " << sortedMotors.size() << std::endl;
            return;
        }

        for (size_t i = 0; i < sortedMotors.size(); i++)
        {
            const auto motor = sortedMotors[i];
            motor->setMotor(cmd[i]);
        }
    }

    void MotorGroup::setMotorsRelativeByIndexUp(const std::vector<MotorControlCmd>& cmd) const
    {
        // 创建按索引排序的电机副本
        std::vector<std::shared_ptr<MotorBase>> sortedMotors = motors_;
        std::sort(sortedMotors.begin(), sortedMotors.end(),
                  [](const auto& a, const auto& b) {
                      return a->getIndex() < b->getIndex();
                  });

        if (cmd.size() < sortedMotors.size())
        {
            std::cerr << "MotorGroup cmd size " << cmd.size() << " error, expected at least " << sortedMotors.size() << std::endl;
            return;
        }

        for (size_t i = 0; i < sortedMotors.size(); i++)
        {
            const auto motor = sortedMotors[i];
            motor->setMotorRelative(cmd[i]);
        }
    }

    void MotorGroup::protectMotor() const
    {
        for (int i = 0; i < size_; i++)
        {
            const auto motor = motors_[i];
            motor->protectMotor();
        }
    }

    std::shared_ptr<MotorBase> MotorGroup::getMotor(const std::string& name) const
    {
        auto it = motorsMap_.find(name);
        if (it == motorsMap_.end())
        {
            std::cerr << "group not such motor:" << name << std::endl;
            return nullptr;
        }
        return it->second;
    }

}
