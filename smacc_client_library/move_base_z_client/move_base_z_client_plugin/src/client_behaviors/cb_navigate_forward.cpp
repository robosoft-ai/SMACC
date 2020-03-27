#include <move_base_z_client_plugin/client_behaviors/cb_navigate_forward.h>

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

    ROS_INFO_STREAM("Straight motion distance: " << dist);

    // TODO: user better:   auto pose = robot->getComponent<cl_move_base_z::Pose>()->get();
    
    //this should work better with a coroutine and await
    ros::Rate rate(10.0);
    tf::StampedTransform currentPose;

    while (ros::ok())
    {
        try
        {
            listener.lookupTransform("/odom", "/base_link",
                                     ros::Time(0), currentPose);

            break;
        }
        catch (tf::TransformException ex)
        {
            ROS_INFO("[CbNavigateFordward] Waiting transform: %s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    tf::Transform forwardDeltaTransform;
    forwardDeltaTransform.setIdentity();
    forwardDeltaTransform.setOrigin(tf::Vector3(dist, 0, 0));

    tf::Transform targetPose = currentPose * forwardDeltaTransform;

    ClMoveBaseZ::Goal goal;
    goal.target_pose.header.frame_id = "/odom";
    goal.target_pose.header.stamp = ros::Time::now();
    tf::poseTFToMsg(targetPose, goal.target_pose.pose);

    ROS_INFO_STREAM("TARGET POSE FORWARD: " << goal.target_pose.pose);
    ros::Duration(0.5).sleep();

    
    geometry_msgs::PoseStamped currentPoseMsg;
    currentPoseMsg.header.frame_id = "/odom";
    currentPoseMsg.header.stamp = ros::Time::now();
    tf::poseTFToMsg(currentPose, currentPoseMsg.pose);

    this->requiresClient(moveBaseClient_);
    odomTracker_ = moveBaseClient_->getComponent<OdomTracker>();
    odomTracker_->pushPath();

    odomTracker_->setStartPoint(currentPoseMsg);
    odomTracker_->setWorkingMode(WorkingMode::RECORD_PATH);

    auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
    plannerSwitcher->setForwardPlanner();

    moveBaseClient_->sendGoal(goal);
}

void CbNavigateForward::onExit()
{
    this->odomTracker_->setWorkingMode(WorkingMode::IDLE);
}

} // namespace cl_move_base_z