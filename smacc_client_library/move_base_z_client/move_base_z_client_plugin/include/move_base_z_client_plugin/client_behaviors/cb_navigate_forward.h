/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <thread>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>

namespace move_base_z_client
{

class CbNavigateForward : public smacc::SmaccClientBehavior
{
public:
    boost::optional<float> forwardDistance;

    // just a stub to show how to use parameterless constructor
    boost::optional<float> forwardSpeed;

    tf::TransformListener listener;

    ClMoveBaseZ *moveBaseClient_;

    odom_tracker::OdomTracker *odomTracker_;

    CbNavigateForward(float forwardDistance);

    CbNavigateForward();

    virtual void onEntry() override;

    virtual void onExit() override;
};
} // namespace move_base_z_client