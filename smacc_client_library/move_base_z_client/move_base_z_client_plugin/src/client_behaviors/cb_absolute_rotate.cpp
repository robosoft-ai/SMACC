
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

void CbAbsoluteRotate::setLocalPlannerYawTolerance(float newtolerance)
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::Config conf;

    ros::NodeHandle nh("~");
    std::string localPlannerName = "TrajectoryPlannerROS";

    if(spinningPlanner && *spinningPlanner == CbAbsoluteRotate::SpiningPlanner::PureSpinning)
    {
        localPlannerName = "PureSpinningLocalPlanner";
    }

    nh.getParam(localPlannerName+"/yaw_goal_tolerance", this->oldYawTolerance);

    dynamic_reconfigure::DoubleParameter yaw_goal_tolerance;
    yaw_goal_tolerance.name = "yaw_goal_tolerance";
    yaw_goal_tolerance.value = newtolerance;
    conf.doubles.push_back(yaw_goal_tolerance);

    srv_req.config = conf;
    ROS_INFO("[CbAbsoluteRotate] updating yaw tolerance local planner to: %lf, from previous value: %lf ", newtolerance, this->oldYawTolerance);
    ros::service::call( localPlannerName, srv_req, srv_resp);
    ros::spinOnce();
}

void CbAbsoluteRotate::onExit()
{
    if(spinningPlanner && *spinningPlanner == CbAbsoluteRotate::SpiningPlanner::PureSpinning)
    {

    }
    else
    {
        this->setLocalPlannerYawTolerance(this->oldYawTolerance);
    }
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

    this->requiresClient(moveBaseClient_);

    auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
    //this should work better with a coroutine and await
    //this->plannerSwitcher_->setForwardPlanner();
    
    if(spinningPlanner && *spinningPlanner == CbAbsoluteRotate::SpiningPlanner::PureSpinning)
    {
        plannerSwitcher->setPureSpinningPlanner();
    }
    else
    {
        plannerSwitcher->setDefaultPlanners();

        if (yawGoalTolerance)
        {
            this->setLocalPlannerYawTolerance(*yawGoalTolerance);
        }
    }

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

    geometry_msgs::PoseStamped stampedCurrentPoseMsg;
    stampedCurrentPoseMsg.header.frame_id = referenceFrame;
    stampedCurrentPoseMsg.header.stamp = ros::Time::now();
    stampedCurrentPoseMsg.pose = currentPoseMsg;

    this->requiresClient(moveBaseClient_);
    auto odomTracker_ = moveBaseClient_->getComponent<odom_tracker::OdomTracker>();
    odomTracker_->pushPath();

    odomTracker_->setStartPoint(stampedCurrentPoseMsg);
    odomTracker_->setWorkingMode(odom_tracker::WorkingMode::RECORD_PATH);

    ROS_INFO_STREAM("[CbAbsoluteRotate] current pose: " << currentPoseMsg);
    ROS_INFO_STREAM("[CbAbsoluteRotate] goal pose: " << goal.target_pose.pose);
    moveBaseClient_->sendGoal(goal);
}
} // namespace cl_move_base_z
