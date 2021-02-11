
#include <move_base_z_client_plugin/client_behaviors/cb_absolute_rotate.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

namespace cl_move_base_z
{
CbAbsoluteRotate::CbAbsoluteRotate()
{
}

CbAbsoluteRotate::CbAbsoluteRotate(float absoluteGoalAngleDegree, float yaw_goal_tolerance)
{
  this->absoluteGoalAngleDegree = absoluteGoalAngleDegree;

  if (yaw_goal_tolerance >= 0)
  {
    this->yawGoalTolerance = yaw_goal_tolerance;
  }
}

void CbAbsoluteRotate::updateTemporalBehaviorParameters(bool undo)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::Config conf;

  ros::NodeHandle nh;
  std::string nodename = "/move_base";
  std::string localPlannerName;
  dynamic_reconfigure::DoubleParameter yaw_goal_tolerance;
  yaw_goal_tolerance.name = "yaw_goal_tolerance";

  dynamic_reconfigure::DoubleParameter max_vel_theta;
  dynamic_reconfigure::DoubleParameter min_vel_theta;
  bool isRosBasePlanner = !spinningPlanner || *spinningPlanner == SpiningPlanner::Default;
  bool isPureSpinningPlanner = spinningPlanner && *spinningPlanner == SpiningPlanner::PureSpinning;

  if (isPureSpinningPlanner)
  {
    localPlannerName = "PureSpinningLocalPlanner";
    max_vel_theta.name = "max_angular_z_speed";
    min_vel_theta.name = "min_vel_theta";
  }
  else if (isRosBasePlanner)
  {
    localPlannerName = "TrajectoryPlannerROS";
    max_vel_theta.name = "max_vel_theta";
    min_vel_theta.name = "min_vel_theta";
  }

  if (!undo)
  {
    if (yawGoalTolerance)
    {
      // save old yaw tolerance
      nh.getParam(nodename + "/" + localPlannerName + "/yaw_goal_tolerance", oldYawTolerance);
      yaw_goal_tolerance.value = *yawGoalTolerance;
      conf.doubles.push_back(yaw_goal_tolerance);
      ROS_INFO("[CbAbsoluteRotate] updating yaw tolerance local planner to: %lf, from previous value: %lf ",
               *yawGoalTolerance, this->oldYawTolerance);
    }

    if (maxVelTheta)
    {
      if (isRosBasePlanner)
      {
        // save old yaw tolerance
        // nh.getParam(nodename + "/"  + localPlannerName+"/min_vel_theta", oldMinVelTheta);
        nh.getParam(nodename + "/" + localPlannerName + "/max_vel_theta", oldMaxVelTheta);
      }
      max_vel_theta.value = *maxVelTheta;
      min_vel_theta.value = -*maxVelTheta;
      conf.doubles.push_back(max_vel_theta);
      conf.doubles.push_back(min_vel_theta);
      ROS_INFO("[CbAbsoluteRotate] updating max vel theta local planner to: %lf, from previous value: %lf ",
               *maxVelTheta, this->oldMaxVelTheta);
      ROS_INFO("[CbAbsoluteRotate] updating min vel theta local planner to: %lf, from previous value: %lf ",
               -*maxVelTheta, this->oldMinVelTheta);
    }
  }
  else
  {
    if (yawGoalTolerance)
    {
      yaw_goal_tolerance.value = oldYawTolerance;
      conf.doubles.push_back(yaw_goal_tolerance);
      ROS_INFO("[CbAbsoluteRotate] restoring yaw tolerance local planner from: %lf, to previous value: %lf ",
               *yawGoalTolerance, this->oldYawTolerance);
    }

    if (maxVelTheta)
    {
      if (isRosBasePlanner)
      {
        max_vel_theta.value = oldMaxVelTheta;
        min_vel_theta.value = oldMinVelTheta;
      }

      conf.doubles.push_back(max_vel_theta);
      conf.doubles.push_back(min_vel_theta);
      ROS_INFO("[CbAbsoluteRotate] restoring max vel theta local planner from: %lf, to previous value: %lf ",
               *maxVelTheta, this->oldMaxVelTheta);
      ROS_INFO("[CbAbsoluteRotate] restoring min vel theta local planner from: %lf, to previous value: %lf ",
               -(*maxVelTheta), this->oldMinVelTheta);
    }
  }

  srv_req.config = conf;
  bool res;
  do
  {
    std::string servername = nodename + "/" + localPlannerName + "/set_parameters";
    res = ros::service::call(servername, srv_req, srv_resp);
    ROS_INFO_STREAM("[CbAbsoluteRotate] dynamic configure call [" << servername << "]: " << res);
    ros::spinOnce();
    if (!res)
    {
      ros::Duration(0.1).sleep();
      ROS_INFO("[CbAbsoluteRotate] Failed, retrtying call");
    }
  } while (!res);
}

void CbAbsoluteRotate::onExit()
{
  if (spinningPlanner && *spinningPlanner == SpiningPlanner::PureSpinning)
  {
  }
  else
  {
  }

  this->updateTemporalBehaviorParameters(true);
}

void CbAbsoluteRotate::onEntry()
{
  double goal_angle;
  ROS_INFO_STREAM("[CbAbsoluteRotate] Absolute yaw Angle:" << this->absoluteGoalAngleDegree);

  if (!this->absoluteGoalAngleDegree)
  {
    this->getCurrentState()->param("goal_angle", goal_angle, 45.0);
  }
  else
  {
    goal_angle = *this->absoluteGoalAngleDegree;
  }

  auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
  // this should work better with a coroutine and await
  // this->plannerSwitcher_->setForwardPlanner();

  if (spinningPlanner && *spinningPlanner == SpiningPlanner::PureSpinning)
    plannerSwitcher->setPureSpinningPlanner();
  else
    plannerSwitcher->setDefaultPlanners();

  updateTemporalBehaviorParameters(false);

  auto p = moveBaseClient_->getComponent<cl_move_base_z::Pose>();
  auto referenceFrame = p->getReferenceFrame();
  auto currentPoseMsg = p->toPoseMsg();

  ClMoveBaseZ::Goal goal;
  goal.target_pose.header.frame_id = referenceFrame;
  goal.target_pose.header.stamp = ros::Time::now();

  auto currentAngle = tf::getYaw(currentPoseMsg.orientation);
  auto targetAngle = goal_angle * M_PI / 180.0;
  goal.target_pose.pose.position = currentPoseMsg.position;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(targetAngle);

  auto odomTracker_ = moveBaseClient_->getComponent<odom_tracker::OdomTracker>();
  if (odomTracker_ != nullptr)
  {
    odomTracker_->pushPath("PureSpinningToAbsoluteGoalOrientation");
    odomTracker_->setStartPoint(p->toPoseStampedMsg());
    odomTracker_->setWorkingMode(odom_tracker::WorkingMode::RECORD_PATH);
  }

  ROS_INFO_STREAM("[CbAbsoluteRotate] current pose: " << currentPoseMsg);
  ROS_INFO_STREAM("[CbAbsoluteRotate] goal pose: " << goal.target_pose.pose);
  moveBaseClient_->sendGoal(goal);
}
}  // namespace cl_move_base_z
