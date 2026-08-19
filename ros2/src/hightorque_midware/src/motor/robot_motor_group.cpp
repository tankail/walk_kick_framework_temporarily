#include "motor/robot_motor_group.h"

namespace hightorque
{
    RobotMotorGroup::RobotMotorGroup(std::shared_ptr<hightorque_robot::robot> rbPtr, std::vector<int> mapIndex, std::vector<std::string> names) : rbPtr_(rbPtr), mapIndex_(mapIndex), names_(names)
    {
    }

    std::string RobotMotorGroup::getAllMotorGroupsStateString() const
    {
        std::stringstream ss;
        for (const auto& group : motorGroups_)
        {
            ss << group.first << ":";
            ss << group.second->getMotorStateString() << "\n";
        }
        return ss.str();
    }

    size_t RobotMotorGroup::getMotorGroupSizeByName(const std::string& name) const
    {
        auto it = motorGroups_.find(name);
        if (it == motorGroups_.end())
        {
            std::cerr << "RobotMotorGroup" << __FUNCTION__ << " get size not such group :" << name << std::endl;
            return 0;
        }
        return it->second->size();
    }

    std::vector<int> RobotMotorGroup::getMotorGroupIndexByName(const std::string& name) const
    {
        auto it = motorGroups_.find(name);
        if (it == motorGroups_.end())
        {
            std::cerr << "RobotMotorGroup" << __FUNCTION__ << " get index not such group :" << name << std::endl;
            return std::vector<int>();
        }
        return it->second->getIndex();
    }

    std::vector<MotorState> RobotMotorGroup::getMotorGroupStateByName(const std::string& name) const
    {
        auto it = motorGroups_.find(name);
        if (it == motorGroups_.end())
        {
            std::cerr << "RobotMotorGroup" << __FUNCTION__ << " get state not such group :" << name << std::endl;
            return {};
        }
        return it->second->getMotorsBackState();
    }

    std::vector<MotorState> RobotMotorGroup::getMotorGroupStateByIndexUpByName(const std::string& name) const
    {
        auto it = motorGroups_.find(name);
        if (it == motorGroups_.end())
        {
            std::cerr << "RobotMotorGroup" << __FUNCTION__ << " get state not such group :" << name << std::endl;
            return {};
        }
        return it->second->getMotorsBackStateByIndexUp();
    }

    void RobotMotorGroup::setMotorGroupByName(const std::string& name, const std::vector<MotorControlCmd>& cmd) const
    {
        auto it = motorGroups_.find(name);
        if (it == motorGroups_.end())
        {
            std::cerr << "RobotMotorGroup" << __FUNCTION__ << " set motor cmd not such group :" << name << std::endl;
            return;
        }
        it->second->setMotors(cmd);
    }

    void RobotMotorGroup::setMotorGroupRelativeByName(const std::string& name, const std::vector<MotorControlCmd>& cmd) const
    {
        auto it = motorGroups_.find(name);
        if (it == motorGroups_.end())
        {
            std::cerr << "RobotMotorGroup" << __FUNCTION__ << " set motor relative cmd not such group :" << name << std::endl;
            return;
        }
        it->second->setMotorsRelative(cmd);
    }

    void RobotMotorGroup::setMotorGroupByIndexUpByName(const std::string& name, const std::vector<MotorControlCmd>& cmd) const
    {
        auto it = motorGroups_.find(name);
        if (it == motorGroups_.end())
        {
            std::cerr << "RobotMotorGroup " << __FUNCTION__ << " set motor index up not such group :" << name << std::endl;
            return;
        }
        it->second->setMotorsByIndexUp(cmd);
    }

    void RobotMotorGroup::setMotorGroupRelativeByIndexUpByName(const std::string& name, const std::vector<MotorControlCmd>& cmd) const
    {
        auto it = motorGroups_.find(name);
        if (it == motorGroups_.end())
        {
            std::cerr << "RobotMotorGroup " << __FUNCTION__ << " set motor relative index up not such group :" << name << std::endl;
            return;
        }
        it->second->setMotorsRelativeByIndexUp(cmd);
    }

    void RobotMotorGroup::protectMotorGroupByName(const std::string& name) const
    {
        auto it = motorGroups_.find(name);
        if (it == motorGroups_.end())
        {
            std::cerr << "RobotMotorGroup " << __FUNCTION__ << " protect motor not such group :" << name << std::endl;
            return;
        }
        it->second->protectMotor();
    }

    void RobotMotorGroup::keepMotorGroupLastCmdByName(const std::string& name) const
    {
        auto it = motorGroups_.find(name);
        if (it == motorGroups_.end())
        {
            std::cerr << "RobotMotorGroup " << __FUNCTION__ << " set last cmd not such group :" << name << std::endl;
            return;
        }
        it->second->keepMotorsLastCmd();
    }

    void RobotMotorGroup::foreachMotorByName(const std::string& name, std::function<bool(const std::shared_ptr<MotorBase>& motor)> action) const
    {
        auto it = motorGroups_.find(name);
        if (it == motorGroups_.end())
        {
            std::cerr << "RobotMotorGroup " << __FUNCTION__ << " foreach motor not such group :" << name << std::endl;
            return;
        }
        for (const auto& motor : it->second->getMotors())
        {
            if (action(motor))
            {
                break;
            }
        }
    }

    std::shared_ptr<MotorBase> RobotMotorGroup::getMotorByName(const std::string& name) const
    {
        auto all = motorGroups_.find("all");
        return all->second->getMotor(name);
    }

}
