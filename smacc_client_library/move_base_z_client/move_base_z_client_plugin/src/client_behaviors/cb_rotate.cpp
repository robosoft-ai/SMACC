#include <move_base_z_client_plugin/client_behaviors/cb_rotate.h>

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

    this->requiresClient(moveBaseClient_);

    auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
    //this should work better with a coroutine and await
    //this->plannerSwitcher_->setForwardPlanner();
    plannerSwitcher->setDefaultPlanners();

    ros::Rate rate(10.0);
    geometry_msgs::Pose currentPoseMsg;
    while (ros::ok())
    {
        tf::StampedTransform currentPose;
        try
        {
            listener.lookupTransform("/odom", "/base_link",
                                     ros::Time(0), currentPose);

            tf::poseTFToMsg(currentPose, currentPoseMsg);
            break;
        }
        catch (tf::TransformException ex)
        {
            ROS_INFO("[CbRotate] Waiting transform: %s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    ClMoveBaseZ::Goal goal;
    goal.target_pose.header.frame_id = "/odom";
    goal.target_pose.header.stamp = ros::Time::now();

    auto currentAngle = tf::getYaw(currentPoseMsg.orientation);
    auto targetAngle = currentAngle + angle_increment_degree * M_PI / 180.0;
    goal.target_pose.pose.position = currentPoseMsg.position;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(targetAngle);

    ROS_INFO_STREAM("current pose: " << currentPoseMsg);
    ROS_INFO_STREAM("goal pose: " << goal.target_pose.pose);
    moveBaseClient_->sendGoal(goal);
}

} // namespace cl_move_base_z
