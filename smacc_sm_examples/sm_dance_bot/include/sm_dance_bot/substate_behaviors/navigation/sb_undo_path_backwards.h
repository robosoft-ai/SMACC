#pragma once

#include <smacc/smacc_substate_behavior.h>
#include <smacc_navigation_plugin/move_base_to_goal.h>
#include <smacc_odom_tracker/odom_tracker.h>
#include <nav_msgs/Path.h>

class SbUndoPathBackwards : public smacc::SmaccSubStateBehavior
{
  tf::TransformListener listener;
    
  smacc::SmaccMoveBaseActionClient *moveBaseClient_;

  smacc_odom_tracker::OdomTracker* odomTracker_;

  virtual void onEntry() override
  {
    this->requiresClient(moveBaseClient_);
    this->requiresComponent(odomTracker_);

    nav_msgs::Path forwardpath = this->odomTracker_->getPath();
    //ROS_INFO_STREAM("[UndoPathBackward] Current path backwards: " << forwardpath);

    this->odomTracker_->setWorkingMode(smacc_odom_tracker::WorkingMode::CLEAR_PATH_BACKWARD);
    
    smacc::SmaccMoveBaseActionClient::Goal goal;
    if ( forwardpath.poses.size()>0)
    {
      goal.target_pose = forwardpath.poses.front();
      moveBaseClient_->plannerSwitcher_->setBackwardPlanner();
      moveBaseClient_->sendGoal(goal);
    }
  }    
};
