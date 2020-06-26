#pragma once
namespace sm_moveit_4
{
namespace place_states
{
// STATE DECLARATION
struct StPlaceApproach : smacc::SmaccState<StPlaceApproach, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<MoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StOpenGripper>,
        Transition<MoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StPlaceApproach, ABORT>/*retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        geometry_msgs::Vector3 offset;
        offset.z = -0.12;
        configure_orthogonal<OrArm, CbMoveCartesianRelative>(offset);
    }

    void runtimeConfigure()
    {
        ClMoveGroup *moveGroupClient;
        this->requiresClient(moveGroupClient);
        moveGroupClient->onMotionExecutionSuccedded(&StPlaceApproach::throwSequenceFinishedEvent, this);
    }

    void onExit()
    {
        ClPerceptionSystem *perceptionSystem;
        this->requiresClient(perceptionSystem);

        perceptionSystem->nextCube();
    }
};
} // namespace place_states
} // namespace sm_moveit_4