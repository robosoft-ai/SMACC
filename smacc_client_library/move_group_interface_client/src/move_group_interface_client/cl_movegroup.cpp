/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/cl_movegroup.h>
namespace move_group_interface_client
{

ClMoveGroup::ClMoveGroup(std::string groupName)
    : moveGroupClientInterface(groupName)
{
    ros::WallDuration(10.0).sleep();
}

ClMoveGroup::~ClMoveGroup()
{
}

void ClMoveGroup::postEventMotionExecutionSucceded()
{
    ROS_INFO("[ClMoveGroup] Post Motion Success Event");
    postEventMotionExecutionSucceded_();
}

void ClMoveGroup::postEventMotionExecutionFailed()
{
    ROS_INFO("[ClMoveGroup] Post Motion Failure Event");
    postEventMotionExecutionFailed_();
}

} // namespace move_group_interface_client
