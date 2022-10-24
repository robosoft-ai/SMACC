/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/component.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>

namespace cl_move_group_interface
{

    struct TrajectoryHistoryEntry
    {
        moveit_msgs::RobotTrajectory trajectory;
        moveit_msgs::MoveItErrorCodes result;
        std::string name;
    };

    class CpTrajectoryHistory : public smacc::ISmaccComponent
    {

    public:
        bool getLastTrajectory(int backIndex, moveit_msgs::RobotTrajectory &trajectory);

        bool getLastTrajectory(moveit_msgs::RobotTrajectory &trajectory);

        void pushTrajectory(std::string name, const moveit_msgs::RobotTrajectory &trajectory, moveit_msgs::MoveItErrorCodes result);

    private:
        std::vector<TrajectoryHistoryEntry> trajectoryHistory_;
    };
} // namespace cl_move_group_interface
