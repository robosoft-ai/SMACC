/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <pluginlib/class_list_macros.h>

namespace smacc
{

ClMoveBaseZ::ClMoveBaseZ()
{
    //ROS_INFO("Smacc Move Base Action Client");
}

std::string ClMoveBaseZ::getName() const
{
    return "MOVE BASE ACTION CLIENT";
}

void ClMoveBaseZ::initialize()
{
    plannerSwitcher_ = std::make_shared<planner_switcher::PlannerSwitcher>(*(this->name_));
    SmaccActionClientBase<move_base_msgs::MoveBaseAction>::initialize();
}

ClMoveBaseZ::~ClMoveBaseZ()
{
}
} // namespace smacc

PLUGINLIB_EXPORT_CLASS(smacc::ClMoveBaseZ, smacc::ISmaccClient)