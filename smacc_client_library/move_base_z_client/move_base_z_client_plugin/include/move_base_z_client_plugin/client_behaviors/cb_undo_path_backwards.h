#pragma once

#include <smacc/smacc_client_behavior.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>

namespace move_base_z_client
{

class CbUndoPathBackwards : public smacc::SmaccClientBehavior
{
  tf::TransformListener listener;

  ClMoveBaseZ *moveBaseClient_;

  virtual void onEntry() override;
};
} // namespace move_base_z_client