#include <move_base_z_client_plugin/client_behaviors/cb_rotate.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

namespace cl_move_base_z
{
CbRotate::CbRotate()
{
}

CbRotate::CbRotate(float rotate_degree)
{
    rotateDegree = rotate_degree;
}

void CbRotate::onEntry()
{
    double angle_increment_degree;

    if (!rotateDegree)
    {
        this->getCurrentState()->param("angle_increment_degree", angle_increment_degree, 45.0);
    }
    else
    {
        angle_increment_degree = *rotateDegree;
    }

    auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
    //this should work better with a coroutine and await
    //this->plannerSwitcher_->setForwardPlanner();
    plannerSwitcher->setDefaultPlanners();

    auto p = moveBaseClient_->getComponent<cl_move_base_z::Pose>();
    auto referenceFrame = p->getReferenceFrame();
    auto currentPoseMsg = p->toPoseMsg();

    tf::Transform currentPose;
    tf::poseMsgToTF(currentPoseMsg, currentPose);

    auto odomTracker = moveBaseClient_->getComponent<odom_tracker::OdomTracker>();
    ClMoveBaseZ::Goal goal;
    goal.target_pose.header.frame_id = referenceFrame;
    goal.target_pose.header.stamp = ros::Time::now();

    auto currentAngle = tf::getYaw(currentPoseMsg.orientation);
    auto targetAngle = currentAngle + angle_increment_degree * M_PI / 180.0;
    goal.target_pose.pose.position = currentPoseMsg.position;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(targetAngle);

    geometry_msgs::PoseStamped stampedCurrentPoseMsg;
    stampedCurrentPoseMsg.header.frame_id = referenceFrame;
    stampedCurrentPoseMsg.header.stamp = ros::Time::now();
    stampedCurrentPoseMsg.pose = currentPoseMsg;

    this->requiresClient(moveBaseClient_);
    odomTracker->pushPath();

    odomTracker->setStartPoint(stampedCurrentPoseMsg);
    odomTracker->setWorkingMode(odom_tracker::WorkingMode::RECORD_PATH);

    ROS_INFO_STREAM("current pose: " << currentPoseMsg);
    ROS_INFO_STREAM("goal pose: " << goal.target_pose.pose);
    moveBaseClient_->sendGoal(goal);
}

} // namespace cl_move_base_z
