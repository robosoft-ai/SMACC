/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <moveit_z_client/client_behaviors/cb_move_joints.h>

namespace sm_moveit
{
namespace cl_movegroup
{

CbMoveJoints::CbMoveJoints(const std::map<std::string, double> &jointValueTarget)
    : jointValueTarget_(jointValueTarget)
{
}

CbMoveJoints::CbMoveJoints()
{
}

void CbMoveJoints::onEntry()
{
    this->requiresClient(movegroupClient_);
    auto &moveGroupInterface = movegroupClient_->moveGroupClientInterface;

    moveGroupInterface.setJointValueTarget(jointValueTarget_);

    moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;
    moveGroupInterface.plan(computedMotionPlan);
}

void CbMoveJoints::onExit()
{
}
} // namespace cl_movegroup
} // namespace sm_moveit