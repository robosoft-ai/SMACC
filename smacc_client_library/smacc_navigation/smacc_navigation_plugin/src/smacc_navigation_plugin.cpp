/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc_navigation_plugin/move_base_action_client.h>
#include <pluginlib/class_list_macros.h>
#include <smacc_navigation_plugin/waypoints_navigator_component.h>
#include <smacc_navigation_plugin/impl/move_base_action_client_impl.h>

namespace smacc
{

SmaccMoveBaseActionClient::SmaccMoveBaseActionClient()
{
    //ROS_INFO("Smacc Move Base Action Client");
    waypointsNavigator_ = std::make_shared<smacc::WaypointNavigator>();
}

std::string SmaccMoveBaseActionClient::getName() const
{
    return "MOVE BASE ACTION CLIENT";
}

void SmaccMoveBaseActionClient::initialize()
{
    plannerSwitcher_ = std::make_shared<smacc_planner_switcher::PlannerSwitcher>(*(this->name_));
    SmaccActionClientBase<move_base_msgs::MoveBaseAction>::initialize();
}

SmaccMoveBaseActionClient::~SmaccMoveBaseActionClient()
{
}
} // namespace smacc

PLUGINLIB_EXPORT_CLASS(smacc::SmaccMoveBaseActionClient, smacc::ISmaccClient)