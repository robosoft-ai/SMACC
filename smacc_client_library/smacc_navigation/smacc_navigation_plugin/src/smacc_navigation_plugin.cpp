/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc_navigation_plugin/move_base_to_goal.h>
#include <pluginlib/class_list_macros.h>

namespace smacc
{

SmaccMoveBaseActionClient::SmaccMoveBaseActionClient()
{
    //ROS_INFO("Smacc Move Base Action Client");
    
}

std::string SmaccMoveBaseActionClient::getName() const
{
    return "MOVE BASE ACTION CLIENT";
}

void SmaccMoveBaseActionClient::initialize()
{
    SmaccActionClientBase<SmaccMoveBaseActionClient, move_base_msgs::MoveBaseAction>::initialize();
    plannerSwitcher_ = std::make_shared<smacc_planner_switcher::PlannerSwitcher>(*(this->name_));
}

SmaccMoveBaseActionClient::~SmaccMoveBaseActionClient()
{   
}
}

PLUGINLIB_EXPORT_CLASS( smacc::SmaccMoveBaseActionClient, smacc::ISmaccClient)