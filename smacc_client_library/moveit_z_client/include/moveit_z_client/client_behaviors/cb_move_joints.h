/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <moveit_z_client/cl_movegroup.h>
#include <smacc/smacc_client_behavior.h>
#include <map>
#include <string>

namespace sm_moveit
{
namespace cl_movegroup
{
class CbMoveJoints : public smacc::SmaccClientBehavior
{
private:
    ClMoveGroup *movegroupClient_;

public:
    std::map<std::string, double> jointValueTarget_;

    CbMoveJoints();
    CbMoveJoints(const std::map<std::string, double> &jointValueTarget);
    virtual void onEntry() override;
    virtual void onExit() override;
};
} // namespace cl_movegroup
} // namespace sm_moveit
