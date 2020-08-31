#include <move_base_z_client_plugin/client_behaviors/cb_navigate_global_position.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>

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
        goalPosition = p;
        goalYaw = yaw;
    }

    void CbNavigateGlobalPosition::setGoal(const geometry_msgs::Pose &pose)
    {
        goalPosition = pose.position;
        goalYaw = tf::getYaw(pose.orientation);
    }

    void CbNavigateGlobalPosition::onEntry()
    {
        ROS_INFO("Entering Navigate Global position");


        ROS_INFO("Component requirements completed");

        auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
        plannerSwitcher->setDefaultPlanners();

        auto pose = moveBaseClient_->getComponent<cl_move_base_z::Pose>()->toPoseMsg();
        auto *odomTracker = moveBaseClient_->getComponent<OdomTracker>();

        odomTracker->pushPath();
        odomTracker->setStartPoint(pose);
        odomTracker->setWorkingMode(WorkingMode::RECORD_PATH);

        execute();
    }

    // auxiliar function that defines the motion that is requested to the move_base action server
    void CbNavigateGlobalPosition::execute()
    {
        auto p = moveBaseClient_->getComponent<cl_move_base_z::Pose>();
        auto referenceFrame = p->getReferenceFrame();
        auto currentPoseMsg = p->toPoseMsg();

        ROS_INFO("Sending Goal to MoveBase");
        ClMoveBaseZ::Goal goal;
        goal.target_pose.header.frame_id = referenceFrame;
        goal.target_pose.header.stamp = ros::Time::now();
        readStartPoseFromParameterServer(goal);

        // store the start pose on the state machine storage so that it can
        // be referenced from other states (for example return to radial start)
        this->getStateMachine()->setGlobalSMData("radial_start_pose", goal.target_pose);

        moveBaseClient_->sendGoal(goal);
    }

    void CbNavigateGlobalPosition::readStartPoseFromParameterServer(ClMoveBaseZ::Goal &goal)
    {
        if (!goalPosition)
        {
            this->getCurrentState()->getParam("start_position_x", goal.target_pose.pose.position.x);
            this->getCurrentState()->getParam("start_position_y", goal.target_pose.pose.position.y);
            double yaw;
            this->getCurrentState()->getParam("start_position_yaw", yaw);

            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        else
        {
            goal.target_pose.pose.position = *goalPosition;
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(*goalYaw);
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
