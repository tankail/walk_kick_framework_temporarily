#include "motor/common_motor_group.h"
#include "motor/motor_base.h"

namespace hightorque
{
    CommonMotorGroup::CommonMotorGroup(std::shared_ptr<hightorque_robot::robot> rbPtr,
                                       std::vector<int> mapIndex,
                                       std::vector<std::string> names,
                                       std::vector<double> lowest,
                                       std::vector<double> highest,
                                       std::vector<double> direction,
                                       std::vector<double> offset)
        : RobotMotorGroup(rbPtr, mapIndex, names)
    {
        // 创建全身
        std::vector<std::shared_ptr<HtdwMotor>> allMotors;
        allMotors.reserve(mapIndex.size());

        for (auto i = 0; i < mapIndex.size(); i++)
        {
            auto motor = std::make_shared<HtdwMotor>(rbPtr, mapIndex[i], names[i],
                                                     lowest[mapIndex[i]], highest[mapIndex[i]],
                                                     direction[i], offset[i]);
            std::cout << "CommonMotorGroup create [" << i << "/" << mapIndex.size()
                      << "] index: " << mapIndex[i] << " name: " << names[i]
                      << " motor (" << lowest[mapIndex[i]] << "~" << highest[mapIndex[i]] << ")"
                      << " direction: " << direction[i] << " offset: " << offset[i] << std::endl;

            allMotors.push_back(motor);
        }

        motorGroups_.emplace("all", std::make_shared<MotorGroup>(allMotors.size(), allMotors));
    }
}
