#include <move_base_z_client_plugin/client_behaviors/cb_navigate_forward.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

namespace cl_move_base_z
{
using namespace ::cl_move_base_z::odom_tracker;

CbNavigateForward::CbNavigateForward(float forwardDistance)
{
  this->forwardDistance = forwardDistance;
}

CbNavigateForward::CbNavigateForward()
{
}

void CbNavigateForward::onEntry()
{
  // straight motion distance
  double dist;

  if (!forwardDistance)
  {
    this->getCurrentState()->param("straight_motion_distance", dist, 3.5);
  }
  else
  {
    dist = *forwardDistance;
  }

  ROS_INFO_STREAM("[CbNavigateForward] Straight motion distance: " << dist);

  auto p = moveBaseClient_->getComponent<cl_move_base_z::Pose>();
  auto referenceFrame = p->getReferenceFrame();
  auto currentPoseMsg = p->toPoseMsg();

  if (this->forceInitialOrientation)
  {
    currentPoseMsg.orientation = *forceInitialOrientation;
    ROS_WARN_STREAM("[CbNavigateForward] Forcing initial straight motion orientation: " << currentPoseMsg.orientation);
  }

  tf::Transform currentPose;
  tf::poseMsgToTF(currentPoseMsg, currentPose);

  tf::Transform forwardDeltaTransform;
  forwardDeltaTransform.setIdentity();
  forwardDeltaTransform.setOrigin(tf::Vector3(dist, 0, 0));

  tf::Transform targetPose = currentPose * forwardDeltaTransform;

  ClMoveBaseZ::Goal goal;
  goal.target_pose.header.frame_id = referenceFrame;
  goal.target_pose.header.stamp = ros::Time::now();
  tf::poseTFToMsg(targetPose, goal.target_pose.pose);

  ROS_INFO_STREAM("TARGET POSE FORWARD: " << goal.target_pose.pose);

  geometry_msgs::PoseStamped currentStampedPoseMsg;
  currentStampedPoseMsg.header.frame_id = referenceFrame;
  currentStampedPoseMsg.header.stamp = ros::Time::now();
  tf::poseTFToMsg(currentPose, currentStampedPoseMsg.pose);

  odomTracker_ = moveBaseClient_->getComponent<OdomTracker>();
  odomTracker_->pushPath("StraightNavigationForwards");

  odomTracker_->setStartPoint(currentStampedPoseMsg);
  odomTracker_->setWorkingMode(WorkingMode::RECORD_PATH);

  auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
  plannerSwitcher->setForwardPlanner();

  moveBaseClient_->sendGoal(goal);
}

void CbNavigateForward::onExit()
{
  this->odomTracker_->setWorkingMode(WorkingMode::IDLE);
}

}  // namespace cl_move_base_z
