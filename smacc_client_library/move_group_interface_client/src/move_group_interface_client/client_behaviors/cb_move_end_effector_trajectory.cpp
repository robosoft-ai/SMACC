/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_move_end_effector_trajectory.h>

namespace move_group_interface_client
{
    CbMoveEndEffectorTrajectory::CbMoveEndEffectorTrajectory()
    {
    }

    void CbMoveEndEffectorTrajectory::onEntry()
    {
        moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;
        // compute inverse kinematics for each pose

        // interpolate speeds

        // interpolate accelerations

        // get current robot state

        // fill plan message

        // call execute

        // handle finishing events
    }
} // namespace move_group_interface_client