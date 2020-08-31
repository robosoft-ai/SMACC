/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include "cb_move_base_client_behavior_base.h"
#include <tf/transform_listener.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>

namespace cl_move_base_z
{
    class CbNavigateForward : public CbMoveBaseClientBehaviorBase
    {
    public:
        boost::optional<float> forwardDistance;

        // just a stub to show how to use parameterless constructor
        boost::optional<float> forwardSpeed;

        tf::TransformListener listener;

        odom_tracker::OdomTracker *odomTracker_;

        CbNavigateForward(float forwardDistance);

        CbNavigateForward();

        virtual void onEntry() override;

        virtual void onExit() override;
    };
} // namespace cl_move_base_z