/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/smacc_client_behavior.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include   <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>

#include <boost/optional.hpp>
#include <tf/transform_listener.h>
#include <tf/tf.h>

namespace move_base_z_client
{
class CbRotate : public smacc::SmaccClientBehavior
{
public:
    tf::TransformListener listener;

    ClMoveBaseZ *moveBaseClient_;

    boost::optional<float> rotateDegree;

    CbRotate();

    CbRotate(float rotate_degree);

    virtual void onEntry() override;
};
} // namespace move_base_z_client
