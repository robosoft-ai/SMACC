#include <move_base_z_client_plugin/client_behaviors/cb_navigate_global_position.h>

namespace move_base_z_client
{

using namespace ::move_base_z_client::odom_tracker;

CbNavigateGlobalPosition::CbNavigateGlobalPosition()
{
}

CbNavigateGlobalPosition::CbNavigateGlobalPosition(float x, float y, float yaw)
{
    auto p = geometry_msgs::Point();
    p.x = x;
    p.y = y;
    initialPoint = p;
    initialYaw = yaw;
}

void CbNavigateGlobalPosition::onEntry()
{
    ROS_INFO("Entering Navigate Global position");

    // this substate will need access to the "MoveBase" resource or plugin. In this line
    // you get the reference to this resource.
    this->requiresClient(moveBaseClient_);
    auto *odomTracker = moveBaseClient_->getComponent<OdomTracker>();

    ROS_INFO("Component requirements completed");

    auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
    plannerSwitcher->setDefaultPlanners();
    odomTracker->setWorkingMode(WorkingMode::RECORD_PATH_FORWARD);

    goToRadialStart();
}

// auxiliar function that defines the motion that is requested to the move_base action server
void CbNavigateGlobalPosition::goToRadialStart()
{
    ROS_INFO("Sending Goal to MoveBase");
    ClMoveBaseZ::Goal goal;
    goal.target_pose.header.frame_id = "/odom";
    goal.target_pose.header.stamp = ros::Time::now();
    readStartPoseFromParameterServer(goal);

    // store the start pose on the state machine storage so that it can
    // be referenced from other states (for example return to radial start)
    this->stateMachine_->setGlobalSMData("radial_start_pose", goal.target_pose);

    moveBaseClient_->sendGoal(goal);
}

void CbNavigateGlobalPosition::readStartPoseFromParameterServer(ClMoveBaseZ::Goal &goal)
{
    if (!initialPoint)
    {
        this->currentState->getParam("start_position_x", goal.target_pose.pose.position.x);
        this->currentState->getParam("start_position_y", goal.target_pose.pose.position.y);
        double yaw;
        this->currentState->getParam("start_position_yaw", yaw);

        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }
    else
    {
        goal.target_pose.pose.position = *initialPoint;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(*initialYaw);
    }

    ROS_INFO_STREAM("start position read from parameter server: " << goal.target_pose.pose.position);
}

// This is the substate destructor. This code will be executed when the
// workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
void CbNavigateGlobalPosition::onExit()
{
    ROS_INFO("Exiting move goal Action Client");
}

} // namespace move_base_z_client
