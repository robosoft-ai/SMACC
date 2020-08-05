/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/component.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>

struct TrajectoryHistoryEntry
{
    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::MoveItErrorCodes result;
};

namespace cl_move_group_interface
{
    class CpTrajectoryHistory : public smacc::ISmaccComponent
    {

    public:

        bool getLastTrajectory(moveit_msgs::RobotTrajectory &trajectory)
        {
            if (trajectoryHistory_.size() ==0)
            {
                return false;
            }

            trajectory = this->trajectoryHistory_.back().trajectory;
            return true;
        }

        void pushTrajectory(const moveit_msgs::RobotTrajectory &trajectory, moveit_msgs::MoveItErrorCodes result)
        {
            TrajectoryHistoryEntry entry;
            this->trajectoryHistory_.push_back(entry);

            auto &last = this->trajectoryHistory_.back();
            last.trajectory = trajectory;
            last.result = result;
        }

    private:
        std::vector<TrajectoryHistoryEntry> trajectoryHistory_;
    };
} // namespace cl_move_group_interface