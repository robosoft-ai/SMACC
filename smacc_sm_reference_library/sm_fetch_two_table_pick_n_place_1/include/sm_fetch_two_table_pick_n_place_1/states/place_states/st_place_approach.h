#pragma once
namespace sm_fetch_two_table_pick_n_place_1
{
namespace place_states
{
// STATE DECLARATION
struct StPlaceApproach : smacc::SmaccState<StPlaceApproach, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvMoveGroupMotionExecutionSucceded<ClMoveGroup, OrArm>, StOpenGripper>,
        Transition<EvMoveGroupMotionExecutionFailed<ClMoveGroup, OrArm>, StPlaceApproach, ABORT>/*retry on failure*/
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
} // namespace sm_fetch_two_table_pick_n_place_1