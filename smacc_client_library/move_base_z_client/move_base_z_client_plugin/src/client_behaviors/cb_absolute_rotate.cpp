
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

    ros::NodeHandle nh;
    nh.getParam("/move_base/BackwardLocalPlanner/yaw_goal_tolerance", this->oldYawTolerance);

    dynamic_reconfigure::DoubleParameter yaw_goal_tolerance;
    yaw_goal_tolerance.name = "yaw_goal_tolerance";
    yaw_goal_tolerance.value = newtolerance;
    conf.doubles.push_back(yaw_goal_tolerance);

    srv_req.config = conf;
    ROS_INFO("updating yaw tolerance local planner to: %lf, from previous value: %lf ", newtolerance, this->oldYawTolerance);
    ros::service::call("/move_base/BackwardLocalPlanner", srv_req, srv_resp);
    ros::spinOnce();
}

void CbAbsoluteRotate::onExit()
{
    if (yawGoalTolerance)
    {

        this->setLocalPlannerYawTolerance(this->oldYawTolerance);
    }
}

void CbAbsoluteRotate::onEntry()
{
    double goal_angle;

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
    plannerSwitcher->setDefaultPlanners();

    if (yawGoalTolerance)
    {
        this->setLocalPlannerYawTolerance(*yawGoalTolerance);
    }

    // TODO: user better:   auto pose = robot->getComponent<cl_move_base_z::Pose>()->get();

    ros::Rate rate(10.0);
    geometry_msgs::Pose currentPoseMsg;
    tf::StampedTransform currentPose;
    while (ros::ok())
    {

        try
        {
            listener.lookupTransform("/odom", "/base_link",
                                     ros::Time(0), currentPose);

            tf::poseTFToMsg(currentPose, currentPoseMsg);
            break;
        }
        catch (tf::TransformException ex)
        {
            ROS_INFO("[CbAbsoluteRotate] Waiting transform: %s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    

    ClMoveBaseZ::Goal goal;
    goal.target_pose.header.frame_id = "/odom";
    goal.target_pose.header.stamp = ros::Time::now();

    auto currentAngle = tf::getYaw(currentPoseMsg.orientation);
    auto targetAngle = goal_angle * M_PI / 180.0;
    goal.target_pose.pose.position = currentPoseMsg.position;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(targetAngle);

    geometry_msgs::PoseStamped stampedCurrentPoseMsg;
    stampedCurrentPoseMsg.header.frame_id = "/odom";
    stampedCurrentPoseMsg.header.stamp = ros::Time::now();
    stampedCurrentPoseMsg.pose = currentPoseMsg;

    this->requiresClient(moveBaseClient_);
    auto odomTracker_ = moveBaseClient_->getComponent<odom_tracker::OdomTracker>();
    odomTracker_->pushPath();

    odomTracker_->setStartPoint(stampedCurrentPoseMsg);
    odomTracker_->setWorkingMode(odom_tracker::WorkingMode::RECORD_PATH);

    ROS_INFO_STREAM("current pose: " << currentPoseMsg);
    ROS_INFO_STREAM("goal pose: " << goal.target_pose.pose);
    moveBaseClient_->sendGoal(goal);
}
} // namespace cl_move_base_z
