#pragma once
namespace sm_fetch_two_table_pick_n_place_1
{
namespace place_states
{
// STATE DECLARATION
struct StOpenGripper : smacc::SmaccState<StOpenGripper, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
            Transition<EvActionSucceeded<ClGripper, OrGripper>, StPlaceRetreat>
            >
            reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrGripper, CbOpenGripper>();
    }

    void runtimeConfigure()
    {
        ClMoveGroup *moveGroupClient;
        this->requiresClient(moveGroupClient);
        // Add again the cube collision

        //moveGroupClient->planningSceneInterface.removeCollisionObjects({"cube_0"});
        //ros::Duration(0.5).sleep();
    }
};
} // namespace place_states
} // namespace sm_fetch_two_table_pick_n_place_1
