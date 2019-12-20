#include <smacc/smacc.h>
#include <tf/transform_datatypes.h>
#include <smacc_navigation_plugin/move_base_action_client.h>

namespace sm_atomic
{
struct CbState1 : public smacc::SmaccClientBehavior
{
    smacc::SmaccMoveBaseActionClient *moveBaseClient_;

    virtual void onEntry() override
    {
        ROS_INFO("Entering State1");
        this->requiresClient(moveBaseClient_);
        goToEndPoint();
    }

    void goToEndPoint()
    {
        geometry_msgs::PoseStamped radialStartPose;

        smacc::SmaccMoveBaseActionClient::Goal goal;
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose = radialStartPose;
        goal.target_pose.pose.position.x = 10;
        goal.target_pose.pose.position.y = 10;
        goal.target_pose.pose.orientation =
            tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI);

        moveBaseClient_->sendGoal(goal);
    }
};
} // namespace sm_three_some