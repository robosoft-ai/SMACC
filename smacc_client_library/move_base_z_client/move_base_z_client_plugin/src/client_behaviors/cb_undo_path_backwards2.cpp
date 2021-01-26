#include <move_base_z_client_plugin/client_behaviors/cb_undo_path_backwards2.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>

namespace cl_move_base_z
{
using namespace ::cl_move_base_z::odom_tracker;

CbUndoPathBackwards2::CbUndoPathBackwards2() : goalLinePassed_(false)
{
}

void CbUndoPathBackwards2::onEntry()
{
  auto *odomTracker = moveBaseClient_->getComponent<OdomTracker>();

  auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
  robotPose_ = moveBaseClient_->getComponent<cl_move_base_z::Pose>();

  nav_msgs::Path forwardpath = odomTracker->getPath();
  // ROS_INFO_STREAM("[UndoPathBackward] Current path backwards: " << forwardpath);

  odomTracker->setWorkingMode(WorkingMode::CLEAR_PATH);

  // this line is used to flush/reset backward planner in the case it were already there
  // plannerSwitcher->setDefaultPlanners();
  if (forwardpath.poses.size() > 0)
  {
    goal.target_pose = forwardpath.poses.front();
    plannerSwitcher->setUndoPathBackwardPlanner();
    moveBaseClient_->sendGoal(goal);
  }
}

void CbUndoPathBackwards2::update()
{
  // check the line was passed
  // y = mx x + y0
  // y = (acos(alpha)/sin(alpha)) (x -x0) + y0
  // sin(alpha)(y - y0) - acos(alpha) (x -x0)= 0 // if greater one side if lower , the other
  auto pose = robotPose_->toPoseMsg();

  auto y = pose.position.y;
  auto x = pose.position.x;
  auto alpha = tf::getYaw(goal.target_pose.pose.orientation);
  auto y0 = goal.target_pose.pose.position.y;
  auto x0 = goal.target_pose.pose.position.x;

  auto evalimplicit = sin(alpha)*(y - y0) - acos(alpha)*(x - x0);
  if (evalimplicit <= 0)
  {
    ROS_INFO("[CbUndoPathBackwards2] goal line passed, stopping behavior and success");
    moveBaseClient_->cancelGoal();
  }
}

void CbUndoPathBackwards2::onExit()
{
  auto *odomTracker = moveBaseClient_->getComponent<OdomTracker>();
  odomTracker->popPath();
}

}  // namespace cl_move_base_z