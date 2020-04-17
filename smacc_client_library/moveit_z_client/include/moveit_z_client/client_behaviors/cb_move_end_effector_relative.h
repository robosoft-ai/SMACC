/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <moveit_z_client/cl_movegroup.h>
#include <smacc/smacc_client_behavior.h>

namespace sm_moveit
{
namespace cl_movegroup
{
class CbMoveEndEffectorRelative : public smacc::SmaccClientBehavior
{
private:
    ClMoveGroup *movegroupClient_;

public:
    geometry_msgs::Transform transform_;

    CbMoveEndEffectorRelative();

    CbMoveEndEffectorRelative(geometry_msgs::Transform transform);

    virtual void onEntry() override;

    virtual void onExit() override;

    void moveRelative(geometry_msgs::Transform &transformOffset);
};
} // namespace cl_movegroup
} // namespace sm_moveit