/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include "cb_move_end_effector_trajectory.h"
#include <move_group_interface_client/components/cp_trajectory_history.h>

namespace cl_move_group_interface
{
    class CbExecuteLastTrajectory : public CbMoveEndEffectorTrajectory
    {
    public:
        CbExecuteLastTrajectory()
        {
        }

        virtual void onEntry() override
        {
            this->requiresClient(movegroupClient_);

            CpTrajectoryHistory *trajectoryHistory;
            this->requiresComponent(trajectoryHistory);

            // this->generateTrajectory();
            // endEffectorTrajectory_ =

            // if (this->endEffectorTrajectory_.size() == 0)
            // {
            //     ROS_WARN_STREAM("[" << smacc::demangleSymbol(typeid(*this).name()) << "] No points in the trajectory. Skipping behavior.");
            //     return;
            // }

            //this->createMarkers();
            //markersInitialized_ = true;

            moveit_msgs::RobotTrajectory trajectory;

            if (trajectoryHistory->getLastTrajectory(trajectory))
            {
                this->executeJointSpaceTrajectory(trajectory);
            }
        }
    };

} // namespace cl_move_group_interface