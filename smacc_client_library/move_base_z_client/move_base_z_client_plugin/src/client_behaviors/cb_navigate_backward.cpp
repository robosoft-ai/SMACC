#include <move_base_z_client_plugin/client_behaviors/cb_navigate_backward.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>
#include <thread>
#include <tf/tf.h>

namespace cl_move_base_z
{
    using namespace ::cl_move_base_z::odom_tracker;

    CbNavigateBackwards::CbNavigateBackwards(float backwardDistance)
    {
        if (backwardDistance < 0)
        {
            ROS_ERROR("cb backward: distance must be greater or equal than 0");
            this->backwardDistance = 0;
        }
        this->backwardDistance = backwardDistance;
    }

    CbNavigateBackwards::CbNavigateBackwards()
    {
    }

    void CbNavigateBackwards::onEntry()
    {
        // straight motion distance
        double dist;

        if (!backwardDistance)
        {
            this->getCurrentState()->param("straight_motion_distance", dist, 3.5);
        }
        else
        {
            dist = *backwardDistance;
        }

        ROS_INFO_STREAM("[CbNavigateBackwards] Straight backwards motion distance: " << dist);

        this->requiresClient(moveBaseClient_);

        auto p = moveBaseClient_->getComponent<cl_move_base_z::Pose>();
        auto referenceFrame = p->getReferenceFrame();
        auto currentPoseMsg = p->toPoseMsg();

        tf::Transform currentPose;
        tf::poseMsgToTF(currentPoseMsg, currentPose);
        tf::Transform forwardDeltaTransform;
        forwardDeltaTransform.setIdentity();
        forwardDeltaTransform.setOrigin(tf::Vector3(-dist, 0, 0));
        tf::Transform targetPose = currentPose * forwardDeltaTransform;
        ClMoveBaseZ::Goal goal;
        goal.target_pose.header.frame_id = referenceFrame;
        goal.target_pose.header.stamp = ros::Time::now();
        tf::poseTFToMsg(targetPose, goal.target_pose.pose);
        ROS_INFO_STREAM("[CbNavigateBackwards] TARGET POSE BACKWARDS: " << goal.target_pose);

        odomTracker_ = moveBaseClient_->getComponent<OdomTracker>();
        if (odomTracker_!=nullptr)
        {
            this->odomTracker_->clearPath();
            this->odomTracker_->setStartPoint(currentPoseMsg);
            this->odomTracker_->setWorkingMode(WorkingMode::RECORD_PATH);
        }

        auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
        plannerSwitcher->setBackwardPlanner();
        // this wait might be needed to let the backward planner receive the last forward message from the odom tracker
        ros::Duration(0.5).sleep();

        moveBaseClient_->sendGoal(goal);
    }

    void CbNavigateBackwards::onExit()
    {
        if (odomTracker_)
        {
            this->odomTracker_->setWorkingMode(WorkingMode::IDLE);
        }
    }

} // namespace cl_move_base_z
