/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/smacc_client_behavior.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>

namespace cl_move_base_z
{

class CbUndoPathBackwards : public smacc::SmaccClientBehavior
{
  tf::TransformListener listener;

  ClMoveBaseZ *moveBaseClient_;

  virtual void onEntry() override;
};
} // namespace cl_move_base_z