#include <move_base_z_client_plugin/client_behaviors/cb_undo_path_backwards2.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>

namespace cl_move_base_z
{
template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

using namespace ::cl_move_base_z::odom_tracker;

CbUndoPathBackwards2::CbUndoPathBackwards2() : goalLinePassed_(false)
{
  triggerThreshold_ = 0.01;  // 10 mm
}

void CbUndoPathBackwards2::onEntry()
{
  odomTracker_ = moveBaseClient_->getComponent<OdomTracker>();

  auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
  robotPose_ = moveBaseClient_->getComponent<cl_move_base_z::Pose>();

  nav_msgs::Path forwardpath = odomTracker_->getPath();
  // ROS_INFO_STREAM("[UndoPathBackward] Current path backwards: " << forwardpath);

  odomTracker_->setWorkingMode(WorkingMode::CLEAR_PATH);

  // this line is used to flush/reset backward planner in the case it were already there
  // plannerSwitcher->setDefaultPlanners();
  if (forwardpath.poses.size() > 0)
  {
    goal_.target_pose = forwardpath.poses.front();
    initial_plane_side_ = evalPlaneSide(robotPose_->toPoseMsg());
    plannerSwitcher->setUndoPathBackwardsPlannerConfiguration();
    moveBaseClient_->sendGoal(goal_);
  }
}

// bool points_on_same_side_of_line(const Vector2d &p1, const Vector2d &p2, const Vector2d &p_line, const Vector2d &normal)
// {
//   return normal.dot(p1 - p_line)*normal.dot(p2 - p_line) > 0.0f;
// }

float CbUndoPathBackwards2::evalPlaneSide(const geometry_msgs::Pose& pose)
{
  // check the line was passed
  // y = mx x + y0
  // y = (sin(alpha)/cos(alpha)) (x -x0) + y0
  // cos(alpha)(y - y0) - sin(alpha) (x -x0)= 0 // if greater one side if lower , the other
  auto y = pose.position.y;
  auto x = pose.position.x;
  auto alpha = tf::getYaw(goal_.target_pose.pose.orientation);
  auto y0 = goal_.target_pose.pose.position.y;
  auto x0 = goal_.target_pose.pose.position.x;

  auto evalimplicit = cos(alpha) * (y - y0) - sin (alpha) * (x - x0);
  return evalimplicit;
}

void CbUndoPathBackwards2::update()
{
  auto pose = robotPose_->toPoseMsg();

  float evalimplicit = evalPlaneSide(pose);
  if (sgn(evalimplicit) != sgn(initial_plane_side_))
  {
    ROS_WARN_STREAM("[CbUndoPathBackwards2] goal_ line passed: "
                                      << evalimplicit << "/" << this->initial_plane_side_);

    if (fabs(evalimplicit) > triggerThreshold_ /*meters*/)
    {
       ROS_WARN_STREAM("[CbUndoPathBackwards2] virtual goal line passed, stopping behavior and success: "
                                      << evalimplicit << "/" << this->initial_plane_side_);
      moveBaseClient_->cancelGoal();
      // this->postSuccessEvent();
      this->postVirtualLinePassed_();
    }
  }
  else
  {
    ROS_INFO_STREAM_THROTTLE(1.0, "[CbUndoPathBackwards2] goal_ line not passed yet: " << evalimplicit << "/"
                                                                                       << this->initial_plane_side_);
  }
}

void CbUndoPathBackwards2::publishMarkers()
{
  double phi = tf::getYaw(goal_.target_pose.pose.orientation);
  visualization_msgs::Marker marker;
  marker.header.frame_id = robotPose_->getReferenceFrame();

  marker.header.stamp = ros::Time::now();
  marker.ns = "my_namespace2";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 3;
  marker.scale.y = 3;
  marker.scale.z = 3;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  marker.pose = goal_.target_pose.pose;

  // geometry_msgs::Point start, end;
  // start.x = pose.position.x;
  // start.y = pose.position.y;

  // end.x = pose.position.x + 0.5 * cos(phi);
  // end.y = pose.position.y + 0.5 * sin(phi);

  // marker.points.push_back(start);
  // marker.points.push_back(end);

  visualization_msgs::MarkerArray ma;
  ma.markers.push_back(marker);

  this->visualizationMarkersPub_.publish(ma);
}

void CbUndoPathBackwards2::onExit()
{
  auto* odomTracker = moveBaseClient_->getComponent<OdomTracker>();
  odomTracker->popPath();
}

}  // namespace cl_move_base_z
