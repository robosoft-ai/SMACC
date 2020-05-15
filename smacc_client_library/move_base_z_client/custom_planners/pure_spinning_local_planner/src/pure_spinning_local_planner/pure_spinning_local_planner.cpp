
#include <angles/angles.h>
#include <pure_spinning_local_planner/pure_spinning_local_planner.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cl_move_base_z::pure_spinning_local_planner::PureSpinningLocalPlanner, nav_core::BaseLocalPlanner)

namespace cl_move_base_z
{
namespace pure_spinning_local_planner
{
// MELODIC
#if ROS_VERSION_MINIMUM(1, 13, 0)
tf::Stamped<tf::Pose> optionalRobotPose(costmap_2d::Costmap2DROS *costmapRos)
{
  geometry_msgs::PoseStamped paux;
  costmapRos->getRobotPose(paux);
  tf::Stamped<tf::Pose> tfpose;
  tf::poseStampedMsgToTF(paux, tfpose);
  return tfpose;
}
#else
// INDIGO AND PREVIOUS
tf::Stamped<tf::Pose> optionalRobotPose(costmap_2d::Costmap2DROS *costmapRos)
{
  tf::Stamped<tf::Pose> tfpose;
  costmapRos->getRobotPose(tfpose);
  return tfpose;
}
#endif

PureSpinningLocalPlanner::PureSpinningLocalPlanner()
{
  ROS_INFO_STREAM("[PureSpinningLocalPlanner] pure spinning planner already created");
}

PureSpinningLocalPlanner::~PureSpinningLocalPlanner()
{
}

bool PureSpinningLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
  goalReached_ = false;
  // ROS_DEBUG("LOCAL PLANNER LOOP");

  tf::Stamped<tf::Pose> tfpose = optionalRobotPose(costmapRos_);

  geometry_msgs::PoseStamped currentPose;
  tf::poseStampedTFToMsg(tfpose, currentPose);
  ROS_INFO_STREAM("[PureSpinningLocalPlanner] current robot pose " << currentPose);

  tf::Quaternion q = tfpose.getRotation();
  auto currentYaw = tf::getYaw(currentPose.pose.orientation);

  while (currentPoseIndex_ < plan_.size())
  {
    auto &goal = plan_[currentPoseIndex_];
    auto targetYaw = tf::getYaw(goal.pose.orientation);

    //double angular_error = angles::shortest_angular_distance(targetYaw, currentYaw);
    double angular_error = targetYaw - currentYaw;

    // all the points must be reached using the control rule, but the last one
    // have an special condition
    if ( (currentPoseIndex_ < plan_.size() -1 && fabs(angular_error) < this->intermediate_goal_yaw_tolerance_)
         || (currentPoseIndex_ == plan_.size() -1 && fabs(angular_error) < this->intermediate_goal_yaw_tolerance_)
    )
    {
      currentPoseIndex_++;
    }
    else
    {
      auto omega = angular_error * k_betta_;
      cmd_vel.angular.z = std::min(std::max(omega, -fabs(max_angular_z_speed_)), fabs(max_angular_z_speed_));


      ROS_DEBUG_STREAM("[PureSpinningLocalPlanner] current yaw: " << currentYaw);
      ROS_DEBUG_STREAM("[PureSpinningLocalPlanner] target yaw: " << targetYaw);
      ROS_DEBUG_STREAM("[PureSpinningLocalPlanner] angular error: " << angular_error);
      ROS_DEBUG_STREAM("[PureSpinningLocalPlanner] param k_betta: " << k_betta_);
      ROS_DEBUG_STREAM("[PureSpinningLocalPlanner] param yaw_goal_tolerance: " << yaw_goal_tolerance_);
      ROS_DEBUG_STREAM("[PureSpinningLocalPlanner] command angular speed: " << cmd_vel.angular.z);
      break;
    }
  }

  ROS_DEBUG_STREAM("[PureSpinningLocalPlanner] completion " << currentPoseIndex_ << "/"<< plan_.size());

  if (currentPoseIndex_ >= plan_.size() -1)
  {
    goalReached_ = true;
  }

  return true;
}

bool PureSpinningLocalPlanner::isGoalReached()
{
  return goalReached_;
}

bool PureSpinningLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
  ROS_DEBUG_STREAM("[PureSpinningLocalPlanner] setting global plan to follow");

  plan_ = plan;
  goalReached_ = false;
  currentPoseIndex_ = 0;
  return true;
}

void PureSpinningLocalPlanner::initialize(std::string name, tf::TransformListener *tf,
                                          costmap_2d::Costmap2DROS *costmapRos)
{
  costmapRos_ = costmapRos;
  this->initialize();
}

void PureSpinningLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmapRos)
{
  costmapRos_ = costmapRos;
  this->initialize();
}

void PureSpinningLocalPlanner::initialize()
{
  ros::NodeHandle nh("~/PureSpinningLocalPlanner");
  nh.param("k_betta", k_betta_, 10.0);
  nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.08);
  nh.param("intermediate_goals_yaw_tolerance", intermediate_goal_yaw_tolerance_, 0.12);
  
  nh.param("max_angular_z_speed", max_angular_z_speed_, 2.0);
}

void publishGoalMarker(double x, double y, double phi)
{
}
}  // namespace pure_spinning_local_planner
}  // namespace cl_move_base_z