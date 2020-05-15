/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <moveit_z_client/cl_movegroup.h>
#include <smacc/smacc_client_behavior.h>

namespace moveit_z_client
{
class CbMoveCartesianRelative : public smacc::SmaccClientBehavior
{
public:
    geometry_msgs::Vector3 offset_;

    boost::optional<double> scalingFactor_;

    CbMoveCartesianRelative();

    CbMoveCartesianRelative(geometry_msgs::Vector3 offset);

    virtual void onEntry() override;

    virtual void onExit() override;

    void moveRelativeCartesian(geometry_msgs::Vector3 &offset);
};
} // namespace moveit_z_client