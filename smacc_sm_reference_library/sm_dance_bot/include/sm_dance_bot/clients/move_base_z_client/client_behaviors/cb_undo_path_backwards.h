#pragma once

#include <smacc/smacc_client_behavior.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <odom_tracker/odom_tracker.h>
#include <nav_msgs/Path.h>

namespace sm_dance_bot
{
namespace move_base_z_client
{
class CbUndoPathBackwards : public smacc::SmaccClientBehavior
{
  tf::TransformListener listener;

  smacc::ClMoveBaseZ *moveBaseClient_;

  virtual void onEntry() override
  {
    this->requiresClient(moveBaseClient_);
    auto *odomTracker = moveBaseClient_->getComponent<odom_tracker::OdomTracker>();

    nav_msgs::Path forwardpath = odomTracker->getPath();
    //ROS_INFO_STREAM("[UndoPathBackward] Current path backwards: " << forwardpath);

    odomTracker->setWorkingMode(odom_tracker::WorkingMode::CLEAR_PATH_BACKWARD);

    smacc::ClMoveBaseZ::Goal goal;
    if (forwardpath.poses.size() > 0)
    {
      goal.target_pose = forwardpath.poses.front();
      moveBaseClient_->plannerSwitcher_->setBackwardPlanner();
      moveBaseClient_->sendGoal(goal);
    }
  }
};
} // namespace move_base_z_client
} // namespace sm_dance_bot