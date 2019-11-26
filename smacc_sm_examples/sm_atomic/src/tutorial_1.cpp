#include <smacc/smacc.h>
#include <smacc_navigation_plugin/move_base_to_goal.h>
#include <tf/transform_datatypes.h>

namespace smacc_tutorial
{

//STATE
class State1;
class State2;
//--------------------------------------------------------------------
class NavigationOrthogonal : public smacc::Orthogonal
{
public:
    virtual void onInitialize() override
    {
        auto *client = this->createClient<smacc::SmaccMoveBaseActionClient>();
        client->name_ = "move_base";
        client->initialize();
    }
};

//--------------------------------------------------------------------
//STATE_MACHINE
struct BaseStateMachine
    : public smacc::SmaccStateMachineBase<BaseStateMachine, State1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<NavigationOrthogonal>();
    }
};

//--------------------------------------------------------------------
struct SbState1 : public smacc::SmaccSubStateBehavior
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
//--------------------------------------------------------------------

struct State1
    : smacc::SmaccState<State1, BaseStateMachine>
{
    typedef mpl::list<smacc::transition<smacc::EvActionSucceded<smacc::SmaccMoveBaseActionClient>, State2>> reactions;

    using SmaccState::SmaccState;

    static void onDefinition()
    {
        static_configure<NavigationOrthogonal, SbState1>();
    }

    void onInitialize()
    {
    }
};
//--------------------------------------------------------------------
struct State2
    : smacc::SmaccState<State2, BaseStateMachine>
{
    using SmaccState::SmaccState;
    void onInitialize()
    {
        ROS_INFO("Entering State2");
    }
};
} //namespace smacc_tutorial

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tutorial");
    ros::NodeHandle nh;

    smacc::run<smacc_tutorial::BaseStateMachine>();
}