#pragma once

#include <smacc/smacc_client_behavior.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <odom_tracker/odom_tracker.h>
#include <nav_msgs/Path.h>

namespace sm_dance_bot
{
class CbUndoPathBackwards : public smacc::SmaccClientBehavior
{
  tf::TransformListener listener;

  smacc::ClMoveBaseZ *moveBaseClient_;

  virtual void onEntry() override
  {
    this->requiresClient(moveBaseClient_);
    auto* odomTracker_ = this->getComponent<odom_tracker::OdomTracker>();

    nav_msgs::Path forwardpath = this->odomTracker_->getPath();
    //ROS_INFO_STREAM("[UndoPathBackward] Current path backwards: " << forwardpath);

    this->odomTracker_->setWorkingMode(odom_tracker::WorkingMode::CLEAR_PATH_BACKWARD);

    smacc::ClMoveBaseZ::Goal goal;
    if (forwardpath.poses.size() > 0)
    {
      goal.target_pose = forwardpath.poses.front();
      moveBaseClient_->plannerSwitcher_->setBackwardPlanner();
      moveBaseClient_->sendGoal(goal);
    }
  }
};
} // namespace sm_dance_bot