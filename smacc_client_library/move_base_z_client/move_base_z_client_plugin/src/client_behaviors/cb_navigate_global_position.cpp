#include <move_base_z_client_plugin/client_behaviors/cb_navigate_global_position.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

namespace cl_move_base_z
{

using namespace ::cl_move_base_z::odom_tracker;

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
    
    ROS_INFO("Component requirements completed");

    auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
    plannerSwitcher->setDefaultPlanners();
    
    auto pose = moveBaseClient_->getComponent<cl_move_base_z::Pose>()->get();

    geometry_msgs::PoseStamped posestamped;
    posestamped.header.frame_id = "/odom";
    posestamped.header.stamp = ros::Time::now();
    posestamped.pose = pose;

    auto *odomTracker = moveBaseClient_->getComponent<OdomTracker>();
    odomTracker->setStartPoint(posestamped);
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
    this->getStateMachine()->setGlobalSMData("radial_start_pose", goal.target_pose);

    moveBaseClient_->sendGoal(goal);
}

void CbNavigateGlobalPosition::readStartPoseFromParameterServer(ClMoveBaseZ::Goal &goal)
{
    if (!initialPoint)
    {
        this->getCurrentState()->getParam("start_position_x", goal.target_pose.pose.position.x);
        this->getCurrentState()->getParam("start_position_y", goal.target_pose.pose.position.y);
        double yaw;
        this->getCurrentState()->getParam("start_position_yaw", yaw);

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

} // namespace cl_move_base_z
