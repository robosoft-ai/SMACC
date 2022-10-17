#pragma once
namespace sm_fetch_six_table_pick_n_sort_1
{
namespace place_states
{
// STATE DECLARATION
struct StPlaceApproach : smacc::SmaccState<StPlaceApproach, SS>
{
    using SmaccState::SmaccState;

    // TRANSITION TABLE
    typedef mpl::list<
        Transition<EvCbSuccess<CbMoveCartesianRelative, OrArm>, StOpenGripper>,
        Transition<EvCbFailure<CbMoveCartesianRelative, OrArm>, StPlaceApproach, ABORT>/*retry on failure*/
        >
        reactions;

    // STATE FUNCTIONS
    static void staticConfigure()
    {
        geometry_msgs::Vector3 offset;
        offset.z = -0.085;
        configure_orthogonal<OrArm, CbMoveCartesianRelative>(offset);
    }

    void runtimeConfigure()
    {
        ros::WallDuration(2).sleep();
    }

    void onExit()
    {
    }
};
} // namespace place_states
} // namespace sm_fetch_six_table_pick_n_sort_1
